#include "selfdrive/modeld/models/nav.h"

#include <cstdio>
#include <cstring>

#include "common/mat.h"
#include "common/modeldata.h"
#include "common/timing.h"


void navmodel_init(NavModelState* s) {
#ifdef USE_ONNX_MODEL
  s->m = new ONNXModel("models/navmodel.onnx", &s->output[0], NAV_NET_OUTPUT_SIZE, USE_DSP_RUNTIME, false, true);
#else
  s->m = new SNPEModel("models/navmodel_q.dlc", &s->output[0], NAV_NET_OUTPUT_SIZE, USE_DSP_RUNTIME, false, true);
#endif

  FILE *f = fopen("models/navmodel.denorm", "r");
  for (int i = 0; i < NAV_DENORM_SIZE; i++) {
    fscanf(f, "%f", &s->denorm[i]);
  }
  fclose(f);
}

NavModelResult* navmodel_eval_frame(NavModelState* s, VisionBuf* buf) {
  memcpy(s->net_input_buf, buf->addr, NAV_INPUT_SIZE);

  double t1 = millis_since_boot();
  s->m->addImage((float*)s->net_input_buf, NAV_INPUT_SIZE/sizeof(float));
  s->m->execute();
  double t2 = millis_since_boot();

  NavModelResult *model_res = (NavModelResult*)&s->output;
  model_res->dsp_execution_time = (t2 - t1) / 1000.;
  return model_res;
}

void fill_plan(cereal::NavModelData::Builder &framed, const NavModelOutputPlan &plan, const NavModelPlanDenorm &denorm) {
  std::array<float, TRAJECTORY_SIZE> pos_x, pos_y;
  std::array<float, TRAJECTORY_SIZE> pos_x_std, pos_y_std;

  for (int i=0; i<TRAJECTORY_SIZE; i++) {
    pos_x[i] = plan.mean[i].x * denorm.scale.mean[i].x + denorm.bias.mean[i].x;
    pos_y[i] = plan.mean[i].y * denorm.scale.mean[i].y + denorm.bias.mean[i].y;
    pos_x_std[i] = exp(plan.std[i].x * denorm.scale.std[i].x + denorm.bias.std[i].x);
    pos_y_std[i] = exp(plan.std[i].y * denorm.scale.std[i].y + denorm.bias.std[i].y);
  }

  auto position = framed.initPosition();
  position.setX(to_kj_array_ptr(pos_x));
  position.setY(to_kj_array_ptr(pos_y));
  position.setXStd(to_kj_array_ptr(pos_x_std));
  position.setYStd(to_kj_array_ptr(pos_y_std));
}

void fill_desire(cereal::NavModelData::Builder &framed, const NavModelOutputDesirePrediction &desire_pred, const NavModelDesireDenorm &denorm) {
  std::array<float, NAV_DESIRE_LEN> desire;
  for (int i = 0; i < NAV_DESIRE_LEN; i++) {
    desire[i] = desire_pred.values[i] * denorm.scale.values[i] + denorm.bias.values[i];
  }
  framed.setDesirePrediction(to_kj_array_ptr(desire));
}

void navmodel_publish(PubMaster &pm, uint32_t frame_id, const NavModelResult &model_res, const NavModelDenorm &model_denorm, float execution_time) {
  // make msg
  MessageBuilder msg;
  auto framed = msg.initEvent().initNavModel();
  framed.setFrameId(frame_id);
  framed.setModelExecutionTime(execution_time);
  framed.setDspExecutionTime(model_res.dsp_execution_time);
  framed.setFeatures(to_kj_array_ptr(model_res.features.values));
  fill_plan(framed, model_res.plan, model_denorm.plan);
  fill_desire(framed, model_res.desire_pred, model_denorm.desire_pred);

  pm.send("navModel", msg);
}

void navmodel_free(NavModelState* s) {
  delete s->m;
}
