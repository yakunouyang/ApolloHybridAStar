#include "mex.h"
#include "parking_planner/hybrid_a_star.h"
#include <cstring>

void mexFunction(int nlhs, mxArray *plhs[],
                 int nrhs, const mxArray *prhs[]) {
  if (nrhs < 2)
    mexErrMsgTxt("Wrong number of input arguments.\n");
  if (nlhs > 1)
    mexErrMsgTxt("Too many output arguments.\n");

  ParkingPlannerConfig config;
  config.xy_bounds = { -20.0, 20.0, -20.0, 20.0 };
  VehicleParameter vehicle;

  if(nrhs > 2) {
    const mxArray *mx_config = prhs[2];
    auto nfields = mxGetNumberOfFields(mx_config);
    for(int i = 0; i < nfields; i++) {
      const char *name = mxGetFieldNameByNumber(mx_config, i);
      auto *pval = mxGetFieldByNumber(mx_config, 0, i);
      auto *val = mxGetPr(pval);

      if(strcmp(name, "xy_bounds") == 0) {
        if(mxGetN(pval) != 4) mexErrMsgTxt("xy_bounds size error\n");
        config.xy_bounds = { val[0], val[1], val[2], val[3] };
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

  std::vector<common::math::Circle2d> obs;

  const mxArray *obstacles = prhs[1];
  auto Nobs = mxGetM(obstacles);
  double *obs_ptr = mxGetPr(obstacles);

  obs.reserve(Nobs);
  for(int i = 0; i < Nobs; i++) {
    obs.emplace_back(obs_ptr[i], obs_ptr[1*Nobs+i], obs_ptr[2*Nobs+i]);
  }

  const mxArray *profile = prhs[0];
  double *profile_ptr = mxGetPr(profile);
  planning::HybridAStar planner(config, vehicle, obs);

  planning::HybridAStartResult result;
  if(!planner.Plan(profile_ptr[0], profile_ptr[1], profile_ptr[2], profile_ptr[3], profile_ptr[4], profile_ptr[5], &result)) {
    mexErrMsgTxt("hybrid a-star failed\n");
  }

  size_t Nfe = result.x.size();
  plhs[0] = mxCreateDoubleMatrix(Nfe, 4, mxREAL);
  double *out = mxGetPr(plhs[0]);
  for(size_t i = 0; i < Nfe; i++) {
    out[0*Nfe+i] = result.x[i];
    out[1*Nfe+i] = result.y[i];
    out[2*Nfe+i] = result.phi[i];
    out[3*Nfe+i] = result.v[i];
//    out[4*Nfe+i] = result.steer[i];
//    out[5*Nfe+i] = result.a[i];
//    out[6*Nfe+i] = 0.0; // TODO: omega
  }
}
