#include "mex.h"
#include "parking_planner/hybrid_a_star.h"
#include <cstring>
#include "my_env.h"

void mexFunction(int nlhs, mxArray *plhs[],
                 int nrhs, const mxArray *prhs[]) {
  if (nrhs < 1)
    mexErrMsgTxt("Wrong number of input arguments.\n");
  if (nlhs > 1)
    mexErrMsgTxt("Too many output arguments.\n");

  ParkingPlannerConfig config;
  config.xy_bounds = { -20.0, 20.0, -20.0, 20.0 };
  VehicleParameter vehicle;

  if(nrhs > 1) {
    const mxArray *mx_config = prhs[1];
    auto nfields = mxGetNumberOfFields(mx_config);
    for(int i = 0; i < nfields; i++) {
      const char *name = mxGetFieldNameByNumber(mx_config, i);
      auto *pval = mxGetFieldByNumber(mx_config, 0, i);
      auto *val = mxGetPr(pval);

      if(strcmp(name, "xy_bounds") == 0) {
        if(mxGetN(pval) != 4) mexErrMsgTxt("xy_bounds size error\n");
        config.xy_bounds = { val[0], val[1], val[2], val[3] };
      } else if(strcmp(name, "LW") == 0) {
        std::copy(val, val + mxGetN(pval), std::back_inserter(vehicle.LW));
      } else if(strcmp(name, "LH") == 0) {
        std::copy(val, val + mxGetN(pval), std::back_inserter(vehicle.LH));
      } else if(strcmp(name, "LM") == 0) {
        std::copy(val, val + mxGetN(pval), std::back_inserter(vehicle.LM));
      } else if(strcmp(name, "LN") == 0) {
        std::copy(val, val + mxGetN(pval), std::back_inserter(vehicle.LN));
      } else if(strcmp(name, "LB") == 0) {
        std::copy(val, val + mxGetN(pval), std::back_inserter(vehicle.LB));
      } else if(strcmp(name, "xy_grid_resolution") == 0) {
        config.xy_grid_resolution = val[0];
      } else if(strcmp(name, "phi_grid_resolution") == 0) {
        config.phi_grid_resolution = val[0];
      } else if(strcmp(name, "step_size") == 0) {
        config.step_size = val[0];
      } else if(strcmp(name, "grid_a_star_xy_resolution") == 0) {
        config.grid_a_star_xy_resolution = val[0];
      } else if(strcmp(name, "delta_t") == 0) {
        config.delta_t = val[0];
      } else if(strcmp(name, "wheel_base") == 0) {
        vehicle.wheel_base = val[0];
      } else if(strcmp(name, "front_hang") == 0) {
        vehicle.front_hang = val[0];
      } else if(strcmp(name, "rear_hang") == 0) {
        vehicle.rear_hang = val[0];
      } else if(strcmp(name, "width") == 0) {
        vehicle.width = val[0];
      } else if(strcmp(name, "delta_max") == 0) {
        vehicle.delta_max = val[0];
      } else {
        mexPrintf("unsupported config: %s\n", name);
      }
    }
  }

  vehicle.GenerateDisc();

  MyEnvironment my_env;
  try {
    my_env = json::parse(mxArrayToString(prhs[0]));
  } catch (std::exception &ex) {
    mexPrintf(ex.what());
    mexErrMsgTxt("json error\n");
  }

  for(auto &ob: my_env.obstacles) {
    ob = ob.ExpandByDistance(0.1);
  }

  planning::HybridAStar planner(config, vehicle, my_env.obstacles);
  planning::HybridAStartResult result;
  if(!planner.Plan(
      my_env.start.x(), my_env.start.y(), my_env.start.theta(), my_env.start_trailer,
      my_env.goal.x(), my_env.goal.y(), my_env.goal.theta(), my_env.goal_trailer, &result)) {
    mexErrMsgTxt("hybrid a-star failed\n");
  }

  size_t Nfe = result.x.size();
  plhs[0] = mxCreateDoubleMatrix(Nfe, 3 + result.trailer_phi[0].size(), mxREAL);
  double *out = mxGetPr(plhs[0]);
  for(size_t i = 0; i < Nfe; i++) {
    out[0*Nfe+i] = result.x[i];
    out[1*Nfe+i] = result.y[i];
    out[2*Nfe+i] = result.phi[i];

    for(size_t j = 0; j < result.trailer_phi[i].size(); j++) {
      out[(3+j)*Nfe+i] = result.trailer_phi[i][j];
    }
//    out[4*Nfe+i] = result.steer[i];
//    out[5*Nfe+i] = result.a[i];
//    out[6*Nfe+i] = 0.0; // TODO: omega
  }
}
