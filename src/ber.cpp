#include "ber.hpp"

using namespace std;

struct ber_data_t
{
    shared_ptr<SubchannelSource> subchannel_source;
    shared_ptr<ConvEncoder> conv_encoder;
};

map<int, ber_data_t> ber_data_state;

void ber_calc(
        int id,
        const subchannel_info_t sc,
        const uint8_t *before_viterbi,
        size_t len_before_viterbi,
        const uint8_t *after_viterbi,
        size_t len_after_viterbi)
{
    ber_data_t ber_data;
    try {
        ber_data = ber_data_state[id];
    }
    catch (out_of_range &e) {
        ber_data.subchannel_source = make_shared<SubchannelSource>(sc);

        // Configuring convolutional encoder
        ber_data.conv_encoder = make_shared<ConvEncoder>(len_after_viterbi);

        ber_data_state[id] = ber_data;
    }

    Buffer in_data(len_after_viterbi);
    std::copy(after_viterbi, after_viterbi + len_after_viterbi, in_data.begin());

    Buffer out_data(0);

    ber_data.conv_encoder->process(&in_data, &out_data);
}

