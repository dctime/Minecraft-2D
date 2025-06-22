#include "digit_cnn.h"
#include "digit_cnn_data.h"
#include <stdio.h>
#include "stm324xg_lcd_sklin.h"



static ai_handle g_network = AI_HANDLE_NULL;
static ai_buffer* g_inputs;
static ai_buffer* g_outputs;

AI_ALIGNED(4) static ai_u8 activations[AI_DIGIT_CNN_DATA_ACTIVATIONS_SIZE];

bool init_cnn() {
    ai_error err;
    ai_network_params params;

    // Step 1: Create network once
    err = ai_digit_cnn_create(&g_network, AI_DIGIT_CNN_DATA_CONFIG);
    if (err.type != AI_ERROR_NONE) return false;

    // Step 2: Bind weights and activations
    if (!ai_digit_cnn_data_params_get(&params)) return false;
    AI_BUFFER_ARRAY_ITEM_SET_ADDRESS(&params.map_activations, 0, activations);
    if (!ai_digit_cnn_init(g_network, &params)) return false;

    // Step 3: Store input/output buffers
    g_inputs = ai_digit_cnn_inputs_get(g_network, NULL);
    g_outputs = ai_digit_cnn_outputs_get(g_network, NULL);

    return true;
}

int run_cnn(float* image28x28) {
    // Copy input
    g_inputs[0].data = image28x28;

    // Run inference
    if (ai_digit_cnn_run(g_network, g_inputs, g_outputs) != 1) return -1;

    float* output = (float*)g_outputs[0].data;
    int max_index = 0;
		float circlePro = output[0];
		float squarePro = output[1];
		float trianglePro = output[2];
	
		char c[30];
		LCD_SetFont(&Font12);
		sprintf(c, "%f, %f, %f", circlePro, squarePro, trianglePro);
		LCD_DisplayStringAt(0, 0, c, LEFT_MODE);
	
    float max_value = output[0];
    for (int i = 1; i < 3; ++i) {
        if (output[i] > max_value) {
            max_value = output[i];
            max_index = i;
        }
    }
    return max_index;
}

void free_cnn() {
    if (g_network != AI_HANDLE_NULL) {
        ai_digit_cnn_destroy(g_network);
        g_network = AI_HANDLE_NULL;
    }
}