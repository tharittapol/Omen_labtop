pcl::PassThrough<PointType> ptfilter (true); // Initializing with true will allow us to extract the removed indices
ptfilter.setInputCloud (cloud_in);
ptfilter.setFilterFieldName ("x");
ptfilter.setFilterLimits (0.0, 1000.0);
ptfilter.filter (*indices_x);
// The indices_x array indexes all points of cloud_in that have x between 0.0 and 1000.0
indices_rem = ptfilter.getRemovedIndices ();
// The indices_rem array indexes all points of cloud_in that have x smaller than 0.0 or larger than 1000.0
// and also indexes all non-finite points of cloud_in
ptfilter.setIndices (indices_x);
ptfilter.setFilterFieldName ("z");
ptfilter.setFilterLimits (-10.0, 10.0);
ptfilter.setNegative (true);
ptfilter.filter (*indices_xz);
// The indices_xz array indexes all points of cloud_in that have x between 0.0 and 1000.0 and z larger than 10.0 or smaller than -10.0
ptfilter.setIndices (indices_xz);
ptfilter.setFilterFieldName ("intensity");
ptfilter.setFilterLimits (FLT_MIN, 0.5);
ptfilter.setNegative (false);
ptfilter.filter (*cloud_out);
// The resulting cloud_out contains all points of cloud_in that are finite and have:
// x between 0.0 and 1000.0, z larger than 10.0 or smaller than -10.0 and intensity smaller than 0.5.