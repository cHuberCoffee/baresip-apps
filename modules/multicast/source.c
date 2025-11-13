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
	mtx_t lock;

	struct config_audio *cfg;

	const struct aucodec *ac;
	struct auenc_state *enc;

	char                    *gong_filename;
	const struct ausrc      *src_gong;
	struct ausrc_st         *src_gong_st;
	struct ausrc_prm         gong_prm;
	volatile bool            eof;
	mcsender_eof_h          *eofh;

	const struct ausrc *src_mic;
	struct ausrc_st    *src_mic_st;
	struct ausrc_prm    mic_prm;
	volatile bool       mic_muted;

	struct aubuf *aubuf;
	void *sampv;

	struct auresamp resamp;
	void *sampv_rs;

	struct list filtl;

	struct mbuf *rtp_mb;
	bool rtp_marker;
	uint32_t ts_ext;
	uint32_t ts_base;
	mcsender_send_h *sendh;

	struct {
		thrd_t tid;
		RE_ATOMIC bool run;
	} thr;

	void *arg;
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

	src->enc = mem_deref(src->enc);

	mtx_lock(&src->lock);
	src->gong_filename = mem_deref(src->gong_filename);
	src->src_gong_st = mem_deref(src->src_gong_st);
	src->src_mic_st = mem_deref(src->src_mic_st);
	mtx_unlock(&src->lock);

	src->aubuf = mem_deref(src->aubuf);
	src->sampv = mem_deref(src->sampv);

	src->sampv_rs = mem_deref(src->sampv_rs);

	src->rtp_mb = mem_deref(src->rtp_mb);
	list_flush(&src->filtl);
}


/**
 * Send auframe data via network (RTP)
 *
 * @param src Multicast source
 * @param af  Audio frame
 *
 * @return 0 if success, otherwise errorcode
 */
static int send_af(struct mcsource *src, struct auframe *af)
{
	size_t ext_len = 0, sampc_rtp = 0, len = 0, frame_size = 0;
	uint32_t ts_delta = 0;
	int err = 0;

	if (!src->ac || !src->ac->ench)
		return EINVAL;

	src->rtp_mb->pos = src->rtp_mb->end = STREAM_PRESZ;
	len = mbuf_get_space(src->rtp_mb),
	err = src->ac->ench(src->enc, &src->rtp_marker, mbuf_buf(src->rtp_mb),
			    &len, AUFMT_S16LE, af->sampv, af->sampc);
	if ((err & 0xffff0000) == 0x00010000) {
		ts_delta = err & 0xffff;
		af->sampc = 0;
	}
	else if (err) {
		warning ("mcsource: audio encoding failed %m\n", err);
		goto out;
	}

	src->rtp_mb->pos = STREAM_PRESZ;
	src->rtp_mb->end = STREAM_PRESZ + ext_len + len;
	if (mbuf_get_left(src->rtp_mb)) {
		uint32_t rtp_ts = src->ts_ext & 0xffffffff;
		if (len) {
			err = src->sendh(ext_len, src->rtp_marker, rtp_ts,
					 src->rtp_mb, src->arg);
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
	src->rtp_marker = false;
	return err;
}



/**
 * Pull frames from aubuf & processing
 *
 * @param src  Multicast source
 *
 * @return 0 if success, otherwise errorcode
 */
static int poll_aubuf_tx(struct mcsource *src)
{
	struct auframe af;
	size_t sampc = 0;
	int err = 0;

	mtx_lock(&src->lock);
	if (!src->eof) {
		sampc = (src->gong_prm.srate * src->gong_prm.ch * PTIME) / 1000;
		auframe_init(&af, AUFMT_S16LE, src->sampv, sampc,
			     src->gong_prm.srate, src->gong_prm.ch);
	} else {
		sampc = (src->mic_prm.srate * src->mic_prm.ch * PTIME) / 1000;
		auframe_init(&af, AUFMT_S16LE, src->sampv, sampc,
			     src->mic_prm.srate, src->mic_prm.ch);
	}
	mtx_unlock(&src->lock);

	aubuf_read_auframe(src->aubuf, &af);
	if (af.srate != src->ac->srate) {
		mtx_lock(&src->lock);
		if (src->resamp.ratio == 0) {
			debug ("mcsource: resampling resetup needed "
				"(%u -> %u)\n", af.srate, src->ac->srate);
			err = auresamp_setup(&src->resamp, af.srate, af.ch,
					     src->ac->srate, src->ac->ch);
			if (err) {
				warning ("mcsource: resampler setup failed "
					"%m\n", err);
				mtx_unlock(&src->lock);
				return err;
			}
		}
		mtx_unlock(&src->lock);

		err = auresamp(&src->resamp, src->sampv_rs, &sampc,
			       af.sampv, af.sampc);
		if (err) {
			warning ("mcsource: resampling failed %m\n", err);
			return err;
		}

		auframe_update(&af, src->sampv_rs, sampc, af.timestamp);
	}

	return send_af(src, &af);
}



/**
 * Audio source error handler (file)
 *
 * @param err Error code
 * @param str Error reason
 * @param arg Handler argument
 */
static void src_mic_err_handler(int err, const char *str, void *arg)
{
	struct mcsource *src = arg;
	(void) src;
	(void) str;
	(void) err;
}


/**
 * Audio source read handler (Sound device)
 *
 * @param af  Audio frame
 * @param arg Handler argument
 */
static void src_mic_read_handler(struct auframe *af, void *arg)
{
	struct mcsource *src = arg;
	size_t package_size = (src->ac->srate * src->ac->ch * PTIME / 1000) *
			      aufmt_sample_size(src->mic_prm.fmt);
	int err = 0;

	if (src->mic_muted)
		return;

	err = aubuf_write_auframe(src->aubuf, af);
	if (err)
		warning ("mcsource: mic aubuf_write %m\n", err);

	if (src->cfg->txmode == AUDIO_MODE_POLL) {
		for (int i = 0; i < 16; i++) {
			if (aubuf_cur_size(src->aubuf) < package_size)
				break;
			poll_aubuf_tx(src);
		}
	}

}


/**
 * Audio source error handler (file)
 *
 * @param err Error code
 * @param str Error reason
 * @param arg Handler argument
 */
static void src_gong_err_handler(int err, const char *str, void *arg)
{
	struct mcsource *src = arg;

	if (err == 0) {
		info ("mcsource: src_gong reached EOF: %s\n", str);

		mtx_lock(&src->lock);
		aubuf_flush(src->aubuf);
		auresamp_init(&src->resamp);
		src->eof = true;
		src->mic_muted = false;
		mtx_unlock(&src->lock);

		src->eofh(src->arg);
	}
}


/**
 * Audio source read handler (file)
 *
 * @param af  Audio frame
 * @param arg Handler argument
 */
static void src_gong_read_handler(struct auframe *af, void *arg)
{
	struct mcsource *src = arg;
	size_t package_size = (src->ac->srate * src->ac->ch * PTIME / 1000) *
			      aufmt_sample_size(src->gong_prm.fmt);
	int err = 0;


	if (src->eof)
		return;

	err = aubuf_write_auframe(src->aubuf, af);
	if (err)
		warning ("mcsource: gong aubuf_write %m\n", err);

	if (src->cfg->txmode == AUDIO_MODE_POLL) {
		for (int i = 0; i < 16; i++) {
			if (aubuf_cur_size(src->aubuf) < package_size)
				break;
			poll_aubuf_tx(src);
		}
	}
}


/**
 * Threaded transmission loop
 *
 * @param arg  Transmission argument
 *
 * @return 0 if success, otherwise errorcode
 */
static int tx_thread(void *arg)
{
	struct mcsource *src = arg;
	size_t package_size = (src->ac->srate * src->ac->ch * PTIME / 1000) *
			      aufmt_sample_size(src->mic_prm.fmt);
	uint64_t ts = 0;

	while (re_atomic_rlx(&src->thr.run)) {
		uint64_t now;
		sys_msleep(4);

		if (!re_atomic_rlx(&src->thr.run))
			break;

		now = tmr_jiffies();
		if (!ts)
			ts = now;
		if (ts > now)
			continue;

		if (aubuf_cur_size(src->aubuf) >= package_size)
			poll_aubuf_tx(src);

		ts += PTIME;
	}

	return 0;
}


/**
 * Set the up aucodec object
 *
 * @param src  Multicast source object
 * @param ac   Audio codec object
 *
 * @return 0 if success, otherwise errorcode
 */
static int setup_aucodec(struct mcsource *src, const struct aucodec *ac)
{
	int err = 0;

	if (!src || !ac)
		return EINVAL;

	src->ac = ac;
	if (src->ac->encupdh) {
		struct auenc_param prm;
		prm.bitrate = 0;
		err = src->ac->encupdh(&src->enc, src->ac, &prm, NULL);
	}

	return err;
}


/**
 * Set the up ausrc object for file playing on stream
 *
 * @param src  Multicast source object
 * @param file Absolute file path
 *
 * @return 0 if success, otherwise errorcode
 */
static int setup_ausrc_gong(struct mcsource *src, const char *file)
{
	struct pl plopt;
	char *opt = NULL;
	int err = 0;

	if (!src || !file)
		return EINVAL;

	if (!conf_get(conf_cur(), "file_ausrc", &plopt)) {
		uint32_t srate = 16000;
		uint32_t ch = 1;

		conf_get_u32(conf_cur(), "file_srate", &srate);
		conf_get_u32(conf_cur(), "file_srate", &srate);

		src->gong_prm.fmt = AUFMT_S16LE;
		src->gong_prm.srate = srate;
		src->gong_prm.ch = ch;
		src->gong_prm.ptime = PTIME;

		err = pl_strdup(&opt, &plopt);
		if (err)
			goto out;

		src->src_gong = ausrc_find(baresip_ausrcl(), opt);
		if (!src->src_gong) {
			err = ENOTSUP;
			goto out;
		}

		err = src->src_gong->alloch(&src->src_gong_st, src->src_gong,
					    &src->gong_prm, file,
					    src_gong_read_handler,
					    src_gong_err_handler, src);
	} else {
		return EINVAL;
	}

out:
	mem_deref(opt);
	return err;
}


/**
 * Set the up ausrc object for microphone
 *
 * @param src   Multicast source object
 * @param ausrc Ausrc for microphone object
 *
 * @return 0 if success, otherwise errorcode
 */
static int setup_ausrc_mic(struct mcsource *src, const struct ausrc *ausrc)
{
	int err = 0;

	if (!src || !src->cfg || !ausrc)
		return EINVAL;

	src->mic_prm.fmt   = src->cfg->src_fmt;
	src->mic_prm.srate = src->cfg->srate_src ? src->cfg->srate_src : 16000;
	src->mic_prm.ch    = src->cfg->channels_src ?
			     src->cfg->channels_src : 2;
	src->mic_prm.ptime = PTIME;

	src->src_mic = ausrc;
	if (!src->src_mic_st) {
		err = src->src_mic->alloch(&src->src_mic_st, src->src_mic,
					   &src->mic_prm, src->cfg->src_dev,
					   src_mic_read_handler,
					   src_mic_err_handler, src);
	}

	return err;
}


/**
 * Allocate necessary buffers
 *
 * @param src Multicast source object
 *
 * @return 0 if success, otherwise errorcode
 */
static int setup_buffers(struct mcsource *src)
{
	int sample_bytes = AUDIO_SAMPSZ * aufmt_sample_size(AUFMT_S16LE);
	int err = 0;

	if (!src)
		return EINVAL;

	err = aubuf_alloc(&src->aubuf, sample_bytes, sample_bytes * 30);
	if (err) {
		warning ("mcsource: failed to allocate audio buffer %m\n",
			 err);
		goto out;
	}

	src->sampv = mem_zalloc(sample_bytes, NULL);
	src->sampv_rs = mem_zalloc(sample_bytes, NULL);
	if (!src->sampv || !src->sampv_rs) {
		err = ENOMEM;
		warning ("mcsource: failed to allocate sample buffers %m\n",
			 err);
		goto out;
	}

	src->rtp_mb = mbuf_alloc(STREAM_PRESZ + 4069);
	if (!src->rtp_mb) {
		err = ENOMEM;
		warning("mcsource: failed to allocate rtp package buffer %m\n",
			 err);
	}
out:
	return err;
}


/**
 * Set the up aufilt in applied order
 *
 * @param src     Multicast source
 * @param aufiltl Audio filter list
 *
 * @return 0 if success, otherwise errorcode
 */
static int setup_aufilt(struct mcsource *src, struct list *aufiltl)
{
	struct aufilt_prm prm;
	struct le *le;
	int err = 0;

	if (!src || !src->ac)
		return EINVAL;

	if (!aufiltl || list_isempty(aufiltl))
		return 0;

	prm.srate = src->ac->srate;
	prm.ch = src->ac->ch;
	prm.fmt = src->mic_prm.fmt;
	for (le = list_head(aufiltl); le; le = le->next) {
		struct aufilt *af = le->data;
		struct aufilt_enc_st *encst = NULL;
		void *ctx = NULL;

		if (af->encupdh) {
			err = af->encupdh(&encst, &ctx, af, &prm, NULL);
			if (err) {
				warning ("mcsource: audio filter %s setup "
					"faild %m, continue with next\n",
					af->name, err);
				continue;
			}

			encst->af = af;
			list_append(&src->filtl, &encst->le, encst);
		}
	}

	return 0;
}


/**
 * Startup the transmission.
 *
 * @param src  Multicast source
 *
 * @return 0 if success, otherwise errorcode
 */
static int start_transmitte(struct mcsource *src)
{
	int err = 0;

	if (!src)
		return EINVAL;

	switch(src->cfg->txmode) {
		case AUDIO_MODE_POLL:
			break;
		case AUDIO_MODE_THREAD:
			re_atomic_rlx_set(&src->thr.run, true);
			err = thread_create_name(&src->thr.tid, "mcsource",
						 tx_thread, src);
			if (err) {
				re_atomic_rlx_set(&src->thr.run, false);
				warning ("mcsource: transmittion thread "
					 "startup %m\n", err);
				return err;
			}
			break;
		default:
			warning ("mcsource: txmode not supported: %d\n",
				 src->cfg->txmode);
			return ENOTSUP;
	}

	return err;
}


/**
 * Start multicast source
 *
 * @param srcp      Multicast source ptr
 * @param ac        Audio codec
 * @param gong_file Absolute path to gong file
 * @param sendh     Send handler ptr
 * @param eofh      EOF handler ptr
 * @param arg       Send handler Argument
 *
 * @return 0 if success, otherwise errorcode
 */
int mcsource_start(struct mcsource **srcp, const struct aucodec *ac,
		   const char *gong_file, mcsender_send_h *sendh,
		   mcsender_eof_h *eofh, void *arg)
{
	struct mcsource *mc_src = NULL;
	int err = 0;

	if (!srcp || !ac)
		return EINVAL;

	mc_src = mem_zalloc(sizeof(*mc_src), mcsource_destructor);
	if (!mc_src)
		return ENOMEM;

	err = mtx_init(&mc_src->lock, mtx_plain) != thrd_success;
	if (err) {
		err = ENOMEM;
		goto out;
	}

	mc_src->cfg = &conf_config()->audio;

	debug ("mcsource: setting up audio encoder %s\n", ac->name);
	err = setup_aucodec(mc_src, ac);
	if (err) {
		warning ("mcsource: codec setup failed %m\n", err);
		goto out;
	}

	if (str_isset(gong_file)) {
		debug ("mcsource: setting up file source for pre-gong\n");
		mc_src->mic_muted = true;
		mc_src->eof = false;
		err = str_dup(&mc_src->gong_filename, gong_file);
		if (err) {
			warning("mcsource: filepath copy failed %m\n", err);
			goto out;
		}

		err = setup_ausrc_gong(mc_src, gong_file);
		if (err) {
			warning ("mcsource: file source setup failed %m\n",
				 err);
			goto out;
		}
	}

	debug ("mcsource: setting up audio source\n");
	err = setup_ausrc_mic(mc_src,
			      ausrc_find(baresip_ausrcl(),
			      mc_src->cfg->src_mod));
	if (err) {
		warning ("mcsource: mic source setup failed %m\n", err);
		goto out;
	}

	debug ("mcsource: setting up and allocation of buffers\n");
	err = setup_buffers(mc_src);
	if (err) {
		warning("mcsource: failed to allocate necessary buffers %m\n",
			 err);
		goto out;
	}

	mc_src->sendh = sendh;
	mc_src->eofh = eofh;
	mc_src->arg = arg;
	mc_src->ts_ext = mc_src->ts_base = rand_u32();

	debug ("mcsource: initialize resampler\n");
	auresamp_init(&mc_src->resamp);

	debug ("mcsource: setting up audio filter in applied order\n");
	err = setup_aufilt(mc_src, baresip_aufiltl());
	if (err) {
		warning ("mcsource: audio filter setup failed %m \n", err);
		goto out;
	}

	debug ("mcsource: startup transmission\n");
	err = start_transmitte(mc_src);
	if (err) {
		warning("mcsource: transmission setup failed %m\n", err);
		goto out;
	}
out:
	if (err)
		mem_deref(mc_src);
	else
		*srcp = mc_src;

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
