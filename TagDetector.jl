# remove comments 2:4 for first-time setup 
# using Pkg
# Pkg.add("Images")
# Pkg.add("AprilTags")

using Images
using AprilTags

detector = AprilTagDetector()

#println("Here")

detector.nThreads = 4
detector.quad_decimate = 1.0
detector.quad_sigma = 0.0
detector.refine_edges = 1
detector.decode_sharpening = 0.25 

image = load("exampleTag.PNG")
tags = detector(image)
freeDetector!(detector)

getAprilTagImage(1, AprilTags.tag36h11)