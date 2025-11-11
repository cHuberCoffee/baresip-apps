/**
 * @file source.c
 *
 * Copyright (C) 2021 Commend.com - c.huber@commend.com
 */

#include <re_atomic.h>
#include <re.h>
#include <rem.h>
#include <baresip.h>

#include <stdlib.h>

#include "multicast.h"

#define DEBUG_MODULE "mcsource"
#define DEBUG_LEVEL 6
#include <re_dbg.h>


/**
 * Multicast source struct
 *
 * Contains configuration of the audio source and buffer for the audio data
 */
struct mcsource {
	struct config_audio *cfg;
	struct ausrc_st *ausrc;
	struct ausrc_prm ausrc_prm;
	const struct aucodec *ac;
	struct auenc_state *enc;
	enum aufmt src_fmt;
	enum aufmt enc_fmt;

	void *sampv;
	struct aubuf *aubuf;
	size_t aubuf_maxsz;
	volatile bool aubuf_started;

	struct aufile *af_gong;
	struct aufile_prm prm_gong;
	bool gong_strm_done;

	struct auresamp resamp;
	void *sampv_rs;
	struct list filtl;

	struct mbuf *mb;
	uint32_t ptime;
	uint64_t ts_ext;
	uint32_t ts_base;
	size_t psize;
	bool marker;

	char *module;
	char *device;

	mcsender_send_h *sendh;
	void *arg;

	struct {
		thrd_t tid;
		RE_ATOMIC bool run;
	} thr;
};


static void mcsource_destructor(void *arg)
{
	struct mcsource *src = arg;

	switch (src->cfg->txmode) {
		case AUDIO_MODE_THREAD:
			if (re_atomic_rlx(&src->thr.run)) {
				re_atomic_rlx_set(&src->thr.run, false);
				thrd_join(src->thr.tid, NULL);
			}
		default:
			break;
	}

	src->ausrc = mem_deref(src->ausrc);
	src->aubuf = mem_deref(src->aubuf);
	src->af_gong = mem_deref(src->af_gong);

	list_flush(&src->filtl);

	src->enc      = mem_deref(src->enc);
	src->mb       = mem_deref(src->mb);
	src->sampv    = mem_deref(src->sampv);
	src->sampv_rs = mem_deref(src->sampv_rs);

	src->module   = mem_deref(src->module);
	src->device   = mem_deref(src->device);
}


/**
 * Read and prepare audio file for transmission
 *
 * @param af  Audio frame object
 * @param src Multicast source
 *
 * @return 0 if success, otherwise errorcode
 */
static int process_aufile(struct auframe *af, struct mcsource *src)
{
	size_t n = 0, sampb = 0;
	int err = 0;

	if (!af || !src)
		return EINVAL;

	if (src->prm_gong.srate != src->resamp.irate &&
		src->resamp.irate != 0) {
		debug("mcsource: resetup resampler for audio file "
			"(input %d | output %d)\n",
			src->prm_gong.srate, src->ac->srate);
		err = auresamp_setup(&src->resamp, src->prm_gong.srate,
			src->prm_gong.channels, src->ac->srate, src->ac->ch);
		if (err)
			return err;
	}

	sampb = (src->prm_gong.srate * src->prm_gong.channels * PTIME / 1000)
		* aufmt_sample_size(src->prm_gong.fmt);
	n = sampb;

	if (n > AUDIO_SAMPSZ * aufmt_sample_size(src->enc_fmt)) {
		warning ("mcsource: audio sample buffer too small\n");
		return ENOMEM;
	}

	/* Temporary read a audio frame for the correct timestamp */
	aubuf_read_auframe(src->aubuf, af);
	if (aufile_read(src->af_gong, (uint8_t *)src->sampv, &n) || n == 0) {
		debug("mcsource: audio file EOF (no silence inserted)\n");
		src->gong_strm_done = true;
		src->af_gong = mem_deref(src->af_gong);
	} else if (n < sampb) {
		memset((uint8_t *)src->sampv + n , 0, sampb - n);
		debug("mcsource: audio file EOF\n");
		src->gong_strm_done = true;
		src->af_gong = mem_deref(src->af_gong);
	}

	if (src->resamp.resample) {
		size_t sampc_rs = AUDIO_SAMPSZ;
		err = auresamp(&src->resamp,
			src->sampv_rs, &sampc_rs,
			src->sampv,
			(sampb / aufmt_sample_size(src->prm_gong.fmt)));
		if (err)
			return err;

		auframe_update(af, src->sampv_rs, sampc_rs,
			af->timestamp);
	} else {
		auframe_update(af, src->sampv,
			(sampb / aufmt_sample_size(src->prm_gong.fmt)),
			af->timestamp);
	}

	return 0;
}


/**
 * Read and prepare audio source for transmission
 *
 * @param af  Audio frame object
 * @param src Multicast source
 *
 * @return 0 if success, otherwise errorcode
 */
static int process_mic(struct auframe *af, struct mcsource *src)
{
	int err = 0;

	if (src->src_fmt != AUFMT_S16LE || src->enc_fmt != AUFMT_S16LE) {
		warning ("mcsource: invalid sample formats (%s - %s)\n",
			aufmt_name(src->src_fmt), aufmt_name(src->enc_fmt));
		return EINVAL;
	}

	if (src->ausrc_prm.srate != src->resamp.irate) {
		debug ("mcsource: resetup resampler for audio source"
			" (input %d | output %d)\n",
			src->ausrc_prm.srate, src->ac->srate);
		err = auresamp_setup(&src->resamp, src->ausrc_prm.srate,
			src->ausrc_prm.ch, src->ac->srate, src->ac->ch);
		if (err) {
			warning ("mcsource: resampler setup for audio source"
				"failed %m\n", err);
			return err;
		}
	}

	aubuf_read_auframe(src->aubuf, af);
	if (src->resamp.resample) {
		size_t sampc_rs = AUDIO_SAMPSZ;
		err = auresamp(&src->resamp,
			src->sampv_rs, &sampc_rs,
			af->sampv, af->sampc);

		if (err) {
			warning ("mcsource: resampling of audio source failed"
				" %m\n", err);
			return err;
		}

		auframe_update(af, src->sampv_rs, sampc_rs,
			af->timestamp);
	}

	return err;
}


/**
 * Encode and send audio data via multicast send handler of src
 *
 * @note This function has REAL-TIME properties
 *
 * @param src Multicast source object
 * @param af  Audio frame object
 */
static void encode_rtp_send(struct mcsource *src, struct auframe *af)
{
	size_t frame_size;
	size_t sampc_rtp;
	size_t len;

	size_t ext_len = 0;
	uint32_t ts_delta = 0;
	int err = 0;

	if (!src->ac || !src->ac->ench)
		return;

	src->mb->pos = src->mb->end = STREAM_PRESZ;

	len = mbuf_get_space(src->mb);
	err = src->ac->ench(src->enc, &src->marker, mbuf_buf(src->mb), &len,
		src->enc_fmt, af->sampv, af->sampc);

	if ((err & 0xffff0000) == 0x00010000) {
		ts_delta = err & 0xffff;
		af->sampc = 0;
	}
	else if (err) {
		warning ("mcsource: %s encode error: %d samples (%m)\n",
			src->ac->name, af->sampc, err);
		goto out;
	}

	src->mb->pos = STREAM_PRESZ;
	src->mb->end = STREAM_PRESZ + ext_len + len;

	if (mbuf_get_left(src->mb)) {
		uint32_t rtp_ts = src->ts_ext & 0xffffffff;

		if (len) {
			err = src->sendh(ext_len, src->marker,
				rtp_ts, src->mb, src->arg);
			if (err)
				goto out;
		}

		if (ts_delta) {
			src->ts_ext += ts_delta;
			goto out;
		}
	}

	sampc_rtp = af->sampc * src->ac->crate / src->ac->srate;
	frame_size = sampc_rtp / src->ac->ch;
	src->ts_ext += (uint32_t) frame_size;

  out:
	src->marker = false;
}


/**
 * Poll timed read from audio buffer
 *
 * @note This function has REAL-TIME properties
 *
 * @param src Multicast source object
 */
static void poll_aubuf_tx(struct mcsource *src)
{
	struct auframe af;
	size_t sampc, num_bytes;
	struct le *le;
	uint32_t srate;
	uint8_t ch;
	int err = 0;

	if (!src)
		return;

	num_bytes = src->psize;
	sampc = num_bytes / aufmt_sample_size(src->src_fmt);
	if (src->resamp.resample) {
		srate = src->resamp.irate;
		ch = src->resamp.ich;
	}
	else {
		srate = src->ausrc_prm.srate;
		ch = src->ausrc_prm.ch;
	}

	auframe_init(&af, AUFMT_S16LE, src->sampv, sampc, srate, ch);
	if (src->af_gong) {
		err = process_aufile(&af, src);
		if (err)
			warning ("mcsource: Error while processing audio file"
				" %m\n", err);
	} else {
		err = process_mic(&af, src);
		if (err)
			warning ("mcsource: Error while processing mic data"
				" %m\n", err);
	}

	for (le = src->filtl.head; le; le = le->next) {
		struct aufilt_enc_st *st = le->data;
		if (st->af && st->af->ench)
			err |= st->af->ench(st, &af);
	}

	if (err)
		warning ("mcsource: aufilter encoding error %m\n", err);

	encode_rtp_send(src, &af);
}


/**
 * Audio source error handler
 *
 * @param err Error code
 * @param str Error string
 * @param arg Multicast source object
 */
static void ausrc_error_handler(int err, const char *str, void *arg)
{
	(void) err;
	(void) str;
	(void) arg;
}


/**
 * Audio source read handler
 *
 * @note This function has REAL-TIME properties
 *
 * @param af  Audio frame
 * @param arg Multicast source object
 */
static void ausrc_read_handler(struct auframe *af, void *arg)
{
	struct mcsource *src = arg;

	if (src->src_fmt != af->fmt) {
		warning ("multicast source: ausrc format mismatch: "
			"expected=%d(%s), actual=%d(%s)\n",
			src->src_fmt, aufmt_name(src->src_fmt),
			af->fmt, aufmt_name(af->fmt));
		return;
	}

	aubuf_write_auframe(src->aubuf, af);
	src->aubuf_started = true;

	if (src->cfg->txmode == AUDIO_MODE_POLL) {
		unsigned i;

		for (i = 0; i < 16; i++) {
			if (aubuf_cur_size(src->aubuf) < src->psize)
				break;

			poll_aubuf_tx(src);
		}
	}
}


/**
 * Standalone transmitter thread function
 *
 * @param arg Multicast source object
 *
 * @return NULL
 */
static int tx_thread(void *arg)
{
	struct mcsource *src = arg;
	uint64_t ts = 0;

	while (re_atomic_rlx(&src->thr.run)) {
		uint64_t now;
		sys_msleep(4);

		if (!src->aubuf_started)
			continue;

		if (!re_atomic_rlx(&src->thr.run))
			break;

		now = tmr_jiffies();
		if (!ts)
			ts = now;

		if (ts > now)
			continue;

		if (aubuf_cur_size(src->aubuf) >= src->psize)
			poll_aubuf_tx(src);

		ts += src->ptime;
	}

	return 0;
}


/**
 * Start audio source
 *
 * @param src Multicast source object
 *
 * @return 0 if success, otherwise errorcode
 */
static int start_source(struct mcsource *src)
{
	int err = 0;
	uint32_t srate_dsp;
	uint32_t channels_dsp;
	bool resamp = false;

	if (!src)
		return EINVAL;

	srate_dsp = src->ac->srate;
	channels_dsp = src->ac->ch;

	if (src->cfg->srate_src && src->cfg->srate_src != srate_dsp) {
		resamp = true;
		srate_dsp = src->cfg->srate_src;
	}
	if (src->cfg->channels_src && src->cfg->channels_src != channels_dsp) {
		resamp = true;
		channels_dsp = src->cfg->channels_src;
	}

	if (resamp && !src->sampv_rs) {
		src->sampv_rs = mem_zalloc(
			AUDIO_SAMPSZ * sizeof(int16_t), NULL);
		if (!src->sampv_rs)
			return ENOMEM;

		err = auresamp_setup(&src->resamp, srate_dsp, channels_dsp,
			src->ac->srate, src->ac->ch);
		if (err) {
			warning ("mcsource: could not setup ausrc "
				"resample %m\n", err);
			return err;
		}
	}

	if (!src->ausrc && ausrc_find(baresip_ausrcl(), NULL)) {
		struct ausrc_prm prm;
		size_t sz;

		prm.srate = srate_dsp;
		prm.ch = channels_dsp;
		prm.ptime = src->ptime;
		prm.fmt = src->src_fmt;

		sz = aufmt_sample_size(src->src_fmt);
		src->psize = sz * (prm.srate * prm.ch * prm.ptime / 1000);
		src->aubuf_maxsz = src->psize * 30;
		if (!src->aubuf) {
			err = aubuf_alloc(&src->aubuf, src->psize,
				src->aubuf_maxsz);
			if (err)
				return err;
		}

		err = ausrc_alloc(&src->ausrc, baresip_ausrcl(),
			src->module, &prm, src->device,
			ausrc_read_handler, ausrc_error_handler, src);
		if (err) {
			warning ("mcsource: start_source faild (%s-%s)"
				" %m\n", src->module, src->device, err);
			return err;
		}

		switch (src->cfg->txmode) {
			case AUDIO_MODE_POLL:
				break;
			case AUDIO_MODE_THREAD:
				if (!re_atomic_rlx(&src->thr.run)) {
					re_atomic_rlx_set(&src->thr.run, true);
					err = thread_create_name(&src->thr.tid,
						"mcsource", tx_thread, src);
					if (err) {
						re_atomic_rlx_set(
							&src->thr.run, false);
						return err;
					}
				}
				break;

			default:
				warning ("mcsrouce: tx mode "
					"not supported (%d)\n",
					src->cfg->txmode);
				return ENOTSUP;
		}

		src->ausrc_prm = prm;
		info ("mcsource: source started with sample format %s\n",
			aufmt_name(src->src_fmt));
	}

	return err;
}


/**
 * Setup all available audio filter for the encoder
 *
 * @param src     Multicast source object
 * @param aufiltl List of audio filter
 *
 * @return 0 if success, otherwise errorcode
 */
static int aufilt_setup(struct mcsource *src, struct list *aufiltl)
{
	struct aufilt_prm prm;
	struct le *le;
	int err = 0;

	if (!src->ac)
		return 0;

	if (!list_isempty(&src->filtl))
		return 0;

	prm.srate = src->ac->srate;
	prm.ch = src->ac->ch;
	prm.fmt = src->enc_fmt;

	for (le = list_head(aufiltl); le; le = le->next) {
		struct aufilt *af = le->data;
		struct aufilt_enc_st *encst = NULL;
		void *ctx = NULL;

		if (af->encupdh) {
			err = af->encupdh(&encst, &ctx, af, &prm, NULL);
			if (err) {
				warning("mcsource: error in encoder"
					"audio-filter '%s' %m (%d)\n",
					af->name, err, err);
			}
			else {
				encst->af = af;
				list_append(&src->filtl, &encst->le,
					encst);
			}
		}

		if (err) {
			warning("mcsource: audio-filter '%s' "
				"update failed %m (%d)\n", af->name, err, err);
			break;
		}
	}

	return err;
}


/**
 * Setup audio file for streaming
 *
 * @param src  Multicast source
 * @param gong Absolute path to audio file
 *
 * @return 0 if success, otherwise errorcode
 */
static int aufile_setup(struct mcsource *src, struct pl *gong) {
	struct aufile *af = NULL;
	char *path = NULL;
	int err = 0;

	if (!src || !gong)
		return EINVAL;

	err = pl_strdup(&path, gong);
	if (err)
		return ENOMEM;


	err = aufile_open(&af, &src->prm_gong, path, AUFILE_READ);
	if (err)
		goto out;

	if (src->prm_gong.srate != src->ausrc_prm.srate &&
		(src->prm_gong.srate % 8000) != 0) {
		err = ENOTSUP;
		warning ("mcsource: file samplerate (%d) not supported %m\n",
			src->prm_gong.srate, err);
		goto out;
	}

out:
	if (err)
		mem_deref(af);
	else
		src->af_gong = af;

	mem_deref(path);
	return err;
}


/**
 * Start multicast source
 *
 * @param srcp  Multicast source ptr
 * @param ac    Audio codec
 * @param sendh Send handler ptr
 * @param arg   Send handler Argument
 *
 * @return 0 if success, otherwise errorcode
 */
int mcsource_start(struct mcsource **srcp, const struct aucodec *ac,
	struct pl *gong, mcsender_send_h *sendh, void *arg)
{
	int err = 0;
	struct mcsource *src = NULL;
	struct config_audio *cfg = &conf_config()->audio;

	if (!srcp || !ac)
		return EINVAL;

	src = mem_zalloc(sizeof(*src), mcsource_destructor);
	if (!src)
		return ENOMEM;

	src->cfg = cfg;
	src->sendh = sendh;
	src->arg = arg;

	src->src_fmt = cfg->src_fmt;
	src->enc_fmt = cfg->enc_fmt;
	src->mb = mbuf_alloc(STREAM_PRESZ + 4096);
	src->sampv = mem_zalloc(
		AUDIO_SAMPSZ * aufmt_sample_size(src->enc_fmt), NULL);
	if (!src->mb || !src->sampv) {
		err = ENOMEM;
		goto out;
	}

	auresamp_init(&src->resamp);
	src->ptime = PTIME;
	src->ts_ext = src->ts_base = rand_u16();
	src->marker = true;

	err = str_dup(&src->module, cfg->src_mod);
	err |= str_dup(&src->device, cfg->src_dev);
	if (err)
		goto out;

	src->ac = ac;
	if (src->ac->encupdh) {
		struct auenc_param prm;

		prm.bitrate = 0;

		err = src->ac->encupdh(&src->enc, src->ac, &prm, NULL);
		if (err) {
			warning ("mcsender: alloc enc %m (%d)\n", err, err);
			goto out;
		}
	}

	err = aufilt_setup(src, baresip_aufiltl());
	if (err)
		goto out;

	err = aufile_setup(src, gong);
	if (err) {
		goto out;
	}

	err = start_source(src);
	if (err)
		goto out;

  out:
	if (err)
		mem_deref(src);
	else
		*srcp = src;

	return err;
}


/**
 * Stop one multicast source.
 *
 * @param unused Multicast audio source object
 */
void mcsource_stop(struct mcsource *unused)
{
	(void) unused;
}


/**
 * Initialize everything needed for the source beforhand
 *
 * @return 0 if success, otherwise errorcode
 */
int mcsource_init(void)
{
	return 0;
}


/**
 * Terminate everything needed for the source afterwards
 *
 */
void mcsource_terminate(void)
{
	return;
}
