#ifndef UNITY_CONFIG_H
#define UNITY_CONFIG_H

#ifdef __cplusplus
extern "C" {
#endif

void unity_output_char(char c);
void unity_output_start(void);
void unity_output_complete(void);

#define UNITY_OUTPUT_CHAR(a) unity_output_char(a)
#define UNITY_OUTPUT_START() unity_output_start()
#define UNITY_OUTPUT_COMPLETE() unity_output_complete()

#ifdef __cplusplus
}
#endif

#define UNITY_SUPPORT_64
#define UNITY_POINTER_WIDTH 32

#endif
