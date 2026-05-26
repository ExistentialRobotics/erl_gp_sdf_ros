@0xb18822b92c33b047;

# Cap'n Proto interface that mirrors the ROS2 SdfQuery and OccQuery services
# exposed by sdf_mapping_node, so clients (e.g. Python via pycapnp) can query
# the SDF/occupancy state without going through ROS.

struct Vec3 {
  x @0 :Float64;
  y @1 :Float64;
  z @2 :Float64;
}

struct SdfQueryRequest {
  # World-frame query points. For a 2D map only x and y are used.
  queryPoints @0 :List(Vec3);
}

struct SdfQueryResponse {
  success                 @0 :Bool;
  # Map dimensionality (2 or 3).
  dim                     @1 :UInt8;
  # Mirror of GpSdfMapping::Setting::TestQuery flags so the client knows which
  # of the optional fields below are populated.
  computeGradient         @2 :Bool;
  computeGradientVariance @3 :Bool;
  computeCovariance       @4 :Bool;
  # Length = queryPoints.size().
  signedDistances         @5 :List(Float64);
  # Length = queryPoints.size() when computeGradient is true; empty otherwise.
  # For 2D maps the z component is 0.
  gradients               @6 :List(Vec3);
  # When computeGradientVariance is true: length = n * (dim + 1), laid out as
  # [var_d, var_grad_x, var_grad_y, (var_grad_z)] per point in column-major
  # order. When false: length = n containing only the SDF variance.
  variances               @7 :List(Float64);
  # When computeCovariance is true: length = n * dim * (dim + 1) / 2,
  # column-major upper-triangular per point. Empty otherwise.
  covariances             @8 :List(Float64);
}

struct OccQueryRequest {
  queryPoints     @0 :List(Vec3);
  # "logodd" or "prob".
  mode            @1 :Text;
  computeGradient @2 :Bool;
}

struct OccQueryResponse {
  success         @0 :Bool;
  dim             @1 :UInt8;
  # Filled when success is false.
  reason          @2 :Text;
  # Length = queryPoints.size(). Either log-odds or probability per query
  # point depending on the request's mode.
  results         @3 :List(Float64);
  # Length = queryPoints.size() when computeGradient is true; empty otherwise.
  gradients       @4 :List(Vec3);
}

interface SdfMapper {
  querySdf @0 (request :SdfQueryRequest) -> (response :SdfQueryResponse);
  queryOcc @1 (request :OccQueryRequest) -> (response :OccQueryResponse);
}
