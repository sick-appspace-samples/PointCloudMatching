--[[----------------------------------------------------------------------------
   
  Application Name:
  PointCloudMatching                                                                                                               
                                                                                              
  Summary:
  Sample for PointCloud matching using Mesh from CAD model.
   
  Description:
  This sample shows how the PointCloud.Matching.Halcon.SurfaceMatcher can be
  used to match a Mesh object obtained from a CAD model in a PointCloud.
  Various pre-processing steps are done to reduce the data before matching.
  
  How to run:
  The script is self-contained. Adjust the variable WAIT to reduce the
  between the pre-processing steps.
         
------------------------------------------------------------------------------]]
WAIT = 2000


-- view handle
local v = View.create()

-- decoration for Shape
shapeDeco = View.ShapeDecoration.create()
shapeDeco:setFillColor(255, 20, 147, 150)
shapeDeco:setLineColor(47, 79, 79)
shapeDeco:setLineWidth(2)

-- decoration for Mesh
local meshDeco = View.MeshDecoration.create()
meshDeco:setSurfaceColor(255, 140, 0, 128)
meshDeco:setVisibleFaces("FRONT_AND_BACK")

-------------------------------------------------------------------------------------
-- 1.) Load input data for matching task
local cloud = PointCloud.load("resources/data.ply")
v:clear()
v:addPointCloud(cloud)
v:present("ASSURED")
Script.sleep(WAIT)

-------------------------------------------------------------------------------------
-- 2.) Prepare a bounding box to cut out relevant part of the scene
local transform = Transform.createTranslation3D(0, 120, 400)
local box = Shape3D.createBox(260, 360, 200, transform)
v:addShape(box, shapeDeco)
v:present("ASSURED")
Script.sleep(WAIT)

-------------------------------------------------------------------------------------
-- 3.) Now cut the relevant part of the scene
local inlieres = cloud:cropShape(box)
local cloudCropped = cloud:extractIndices(inlieres)
v:clear()
v:addPointCloud(cloudCropped)
v:present("ASSURED")
Script.sleep(WAIT)

-------------------------------------------------------------------------------------
-- 4.) Fit a plane-model to reduce data further
local shapeFitter = PointCloud.ShapeFitter.create()
shapeFitter:setDistanceThreshold(15)
local planePoints, inlierIndices = shapeFitter:fitPlane(cloudCropped)
-- mark all points in the plane
cloudCropped:setIntensity(inlierIndices, 1)
v:addPointCloud(cloudCropped)
v:present("ASSURED")
Script.sleep(WAIT)

-------------------------------------------------------------------------------------
-- 5.) Remove all inlier points of the plane-fit from the data
local cloudFiltered = cloudCropped:extractIndices(inlierIndices, true)
v:addPointCloud(cloudFiltered)
v:present("ASSURED")

-------------------------------------------------------------------------------------
-- 6.) Prepare a PointCloud.Matching.Halcon.SurfaceMatcher model
local matcher = PointCloud.Matching.Halcon.SurfaceMatcher.create()
matcher:setRelativeSamplingDistance(0.02)
matcher:setKeypointFraction(0.8)
matcher:setMinScore(.2)
matcher:setMaxMatches(1)
matcher:setNormalComputationMethod("MLS")
matcher:setPoseRefinementParameters(0.01, 0.5)

-------------------------------------------------------------------------------------
-- 7.) Load a teach model as Mesh from CAD
local teachMesh = Mesh.load("resources/V2D63xModel.ply")
local teachObjectResampled = matcher:teachMesh(teachMesh, 0.05, 0.01)

-------------------------------------------------------------------------------------
-- 8.) Compute the matching of the teach model
local scores, poses = matcher:match(cloudFiltered)

v:clear()
-- iterate over all matching results
for index,score in ipairs(scores) do
  -- print some statistics on the score and the pose of the matching
  print(score, poses[index]:toString())
  -- transform the resampled teach model from step 7.) according to the pose
  local teachObjectResampledTransformed = teachObjectResampled:transform(poses[index])
  -- identify all points from the scene that are close to the model
  local matchIndices = cloud:findNearbyPoints(teachObjectResampledTransformed, 10)
  -- mark all points from the scene from the matching
  cloud:setIntensity(matchIndices, index/#scores)
  -- display result
  v:addMesh(teachMesh:transform(poses[index]), meshDeco)
end
v:addPointCloud(cloud)
v:present()
