#pragma once

#include "cereal/messaging/messaging.h"
#include "cereal/visionipc/visionipc_client.h"
#include "common/util.h"
#include "common/modeldata.h"
#include "selfdrive/modeld/models/commonmodel.h"
#include "selfdrive/modeld/runners/run.h"

constexpr int NAV_INPUT_SIZE = 256*256;
constexpr int NAV_FEATURE_LEN = 64;
constexpr int NAV_DESIRE_LEN = 32;

struct NavModelOutputXY {
  float x;
  float y;
};
static_assert(sizeof(NavModelOutputXY) == sizeof(float)*2);

struct NavModelOutputPlan {
  std::array<NavModelOutputXY, TRAJECTORY_SIZE> mean;
  std::array<NavModelOutputXY, TRAJECTORY_SIZE> std;
};
static_assert(sizeof(NavModelOutputPlan) == sizeof(NavModelOutputXY)*TRAJECTORY_SIZE*2);

struct NavModelOutputDesirePrediction {
  std::array<float, NAV_DESIRE_LEN> values;
};
static_assert(sizeof(NavModelOutputDesirePrediction) == sizeof(float)*NAV_DESIRE_LEN);

struct NavModelOutputFeatures {
  std::array<float, NAV_FEATURE_LEN> values;
};
static_assert(sizeof(NavModelOutputFeatures) == sizeof(float)*NAV_FEATURE_LEN);

struct NavModelResult {
  const NavModelOutputPlan plan;
  const NavModelOutputDesirePrediction desire_pred;
  const NavModelOutputFeatures features;
  float dsp_execution_time;
};
static_assert(sizeof(NavModelResult) == sizeof(NavModelOutputPlan) + sizeof(NavModelOutputDesirePrediction) + sizeof(NavModelOutputFeatures) + sizeof(float));

// Denorm constants
struct NavModelPlanDenorm {
  const NavModelOutputPlan scale;
  const NavModelOutputPlan bias;
};
static_assert(sizeof(NavModelPlanDenorm) == sizeof(NavModelOutputPlan) * 2);

struct NavModelDesireDenorm {
  const NavModelOutputDesirePrediction scale;
  const NavModelOutputDesirePrediction bias;
};
static_assert(sizeof(NavModelDesireDenorm) == sizeof(NavModelOutputDesirePrediction) * 2);

struct NavModelDenorm {
  const NavModelPlanDenorm plan;
  const NavModelDesireDenorm desire_pred;
};
static_assert(sizeof(NavModelDenorm) == sizeof(NavModelPlanDenorm) + sizeof(NavModelDesireDenorm));

// Model state
constexpr int NAV_OUTPUT_SIZE = sizeof(NavModelResult) / sizeof(float);
constexpr int NAV_NET_OUTPUT_SIZE = NAV_OUTPUT_SIZE - 1;
constexpr int NAV_DENORM_SIZE = sizeof(NavModelDenorm) / sizeof(float);

struct NavModelState {
  RunModel *m;
  uint8_t net_input_buf[NAV_INPUT_SIZE];
  float output[NAV_OUTPUT_SIZE];
  float denorm[NAV_DENORM_SIZE];
};

void navmodel_init(NavModelState* s);
NavModelResult* navmodel_eval_frame(NavModelState* s, VisionBuf* buf);
void navmodel_publish(PubMaster &pm, uint32_t frame_id, const NavModelResult &model_res, const NavModelDenorm &model_denorm, float execution_time);
void navmodel_free(NavModelState* s);
