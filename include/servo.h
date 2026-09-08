#ifndef STEER_H_
#define STEER_H_

#ifdef __cplusplus
extern "C" {
#endif

#define MAX_STEER 70.0f
void Steer(float angle);
void TestServo();

#ifdef __cplusplus
}
#endif

#endif
