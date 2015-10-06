#ifndef __BER_H_
#define __BER_H_

#include "dab.h"
#include "conv/ModCodec.h"
#include "conv/PcDebug.h"
#include "conv/ConvEncoder.h"
#include "conv/PuncturingRule.h"
#include "conv/PuncturingEncoder.h"
#include "conv/SubchannelSource.h"
#include <map>
#include <memory>

void ber_calc(
        int id,
        struct subchannel_info_t sc,
        const uint8_t *before_viterbi,
        size_t len_before_viterbi,
        const uint8_t *after_viterbi,
        size_t len_after_viterbi);

#endif
