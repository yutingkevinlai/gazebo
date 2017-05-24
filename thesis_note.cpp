



"4.Feature Extraction and Individual Feature Saliency"

	key point				-> 3D Harris key point detection	(others: SIFT)
	fature descriptors	-> Fast Point Feature Histogram	(others: Shape Context)

	extract key points for both ideal and noisy 3D models -> calculate "keypoint distance" using KLD(Kullback-Leibler Divergence) -> use sigmoid function to warp KLD into range [0,1] to indicate saliency.

"5.The Virtual RGBD Sensor"
	1) Target RGBD Sensor
	2) Extract Depth Image
	3) Simulation Methods
		a) The Occlusion Effect: rising edge
		b) Intensity Saturation of Project Light
			appreance of an object: ambient color, diffuse color, specular color
			> rough surface: high diffuse, low specular
			> smooth surface: high specular, low diffuse
		c) Low Confidence Effect
			the light projected on a steep surface causes deformations of the structural light + the light is reflected away from the IR camera = affect depth value
		d) Noise Synthesis
			(1) capture a depth image of a plane, perpendicular to the view ray
			(2) extract sensor noise fromt he captured depth image
			(3) apply 2D Discrete Fourier Transform on sensor noise
			(4) preserve the magnitude of the spectrum except for low frequencies, and generate the random phase of the spectrum
			(5) generate noise using inverse 2D DFT of the original magnitude and random phase
		e) Blurring Effect
			> caused by the internal stage of the depth retrieving algorithm
			> apply bilateral filter to the depth image to prevent undesired result and obtain smoothing effect
		f) Disturbance on Threshold
			> utilize perlin noise to generate more realistic noise

"6.Performance of the Evaluation Platform"
	1) Time to the Steady State
	2) Virtual Depth Sensor Capturing Time
	3) Computation Time
		a) stacking time
		b) sensor capturing time
		c) pose estimation = number of estimated objects * time for estimating one object

"7.Experiments"
	1) Ideal Segmentation
	2) Evaluating Pose Estimation
		a) single model pose estimation
			Success Count Before Failure has two peaks -> the same as the real condition
			> first peak shows the normal distribution (monotonically decreasing)
			> second peak shows the high success rate of the upper layer
		b) multiple model pose estimation
	3) Optimizing Parameters Using the Evaluation Platform
		a) need at least 560 trials to determine success rate
		b) optimal SHOT search radius: 8 (mm), key points are at least 5 (mm)
		c) similarity threshold: 0.65

"8.Conclusion"
	
			




