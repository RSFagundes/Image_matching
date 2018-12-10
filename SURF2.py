import cv2
import cv
import numpy
from naoqi import ALProxy
import sys
import Image
import numpy as np

def SURF(queryImg,trainImg, surf):
	queryGray = cv2.cvtColor(queryImg, cv2.COLOR_BGR2GRAY)
	trainGray = cv2.cvtColor(trainImg, cv2.COLOR_BGR2GRAY)

	(Tkeypoints, Tdescriptors) = surf.detectAndCompute(trainGray, None)
	(Qkeypoints, Qdescriptors) = surf.detectAndCompute(queryGray, None)

	# extract vectors of size 64 from raw descriptors numpy arrays
	rowsize = len(Tdescriptors) / len(Tkeypoints)
	if rowsize > 1:
		  Trows = numpy.array(Tdescriptors, dtype = numpy.float32).reshape((-1, rowsize))
		  Qrows = numpy.array(Qdescriptors, dtype = numpy.float32).reshape((-1, rowsize))
		  #print hrows.shape, nrows.shape
	else:
		  Trows = numpy.array(Tdescriptors, dtype = numpy.float32)
		  Qrows = numpy.array(Qdescriptors, dtype = numpy.float32)
		  rowsize = len(Trows[0])

	# kNN training - learn mapping from hrow to hkeypoints index
	samples = Trows
	responses = numpy.arange(len(Tkeypoints), dtype = numpy.float32)
	#print len(samples), len(responses)
	knn = cv2.KNearest()
	knn.train(samples,responses)

	# retrieve index and value through enumeration
	for i, descriptor in enumerate(Qrows):
		  descriptor = numpy.array(descriptor, dtype = numpy.float32).reshape((1, rowsize))
		  #print i, descriptor.shape, samples[0].shape
		  retval, results, neigh_resp, dists = knn.find_nearest(descriptor, 1)
		  res, dist =  int(results[0][0]), dists[0][0]
		  #print res, dist

		  if dist < 0.1:
		      # draw matched keypoints in red color
		      color = (0, 0, 255)
		  else:
		      # draw unmatched in blue color
		      color = (255, 0, 0)
		  # draw matched key points on haystack image
		  x,y = Tkeypoints[res].pt
		  center = (int(x),int(y))
		  cv2.circle(trainImg,center,2,color,-1)
		  # draw matched key points on needle image
		  x,y = Qkeypoints[i].pt
		  center = (int(x),int(y))
		  cv2.circle(queryImg,center,2,color,-1)

	cv2.imshow('Train',trainImg)
	cv2.imshow('Query',queryImg)
	cv2.waitKey(1)
	#cv2.destroyAllWindows()

	return 1,1

if __name__ =="__main__":

	queryDatabase = []
	queryDatabase.append("/media/DATA/ENSTA_Bretagne/2013.2/5.5/Vision/Projet/Prise4/query.JPG")
	queryImg = cv2.imread(queryDatabase[0])

	# Main program NAO
	IP = "172.20.11.146"  # Replace here with your NaoQi's IP address. 172.20;11238
	PORT = 9559
	names  = ["HeadYaw", "HeadPitch"]
	# Read IP address from first argument if any.
	if len(sys.argv) > 1:
		 IP = sys.argv[1]
	# init motion
	try:
		  motionProxy = ALProxy("ALMotion", IP, PORT)
	except Exception, e:
		  print "Could not create proxy to ALMotion"
		  print "Error was: ", e

	# work ! set current to servos
	stiffnesses  = 1.0
	motionProxy.setStiffnesses(names, stiffnesses)
	# init video
	cameraProxy = ALProxy("ALVideoDevice", IP, PORT)
	resolution = 1    # 0 : QQVGA, 1 : QVGA, 2 : VGA
	colorSpace = 11   # RGB
	camNum = 1 # 0:top cam, 1: bottom cam
	fps = 1; # frame Per Second
	cameraProxy.setParam(18, camNum)
	try:
		 videoClient = cameraProxy.subscribe("python_client",
		                                     resolution, colorSpace, fps)
	except:
		 cameraProxy.unsubscribe("python_client")
		 videoClient = cameraProxy.subscribe("python_client",
		                                     resolution, colorSpace, fps)
	print "videoClient ",videoClient
	# Get a camera image.
	# image[6] contains the image data passed as an array of ASCII chars.
	naoImage = cameraProxy.getImageRemote(videoClient)
	imageWidth = naoImage[0]
	imageHeight = naoImage[1]

	found = True

	# build feature detector and descriptor extractor
	hessian_threshold = 400
	surf = cv2.SURF(hessian_threshold)
	try:
		 while found:

				# Get current image (top cam)
				naoImage = cameraProxy.getImageRemote(videoClient)

				# Get the image size and pixel array.
				imageWidth = naoImage[0]
				imageHeight = naoImage[1]
				array = naoImage[6]

				print imageWidth,"x",imageHeight
				

				# Create a PIL Image from our pixel array.
				pilImg = Image.fromstring("RGB", (imageWidth, imageHeight), array)
				# Convert Image to OpenCV
				cvImg = np.array(pilImg)
				cvImg = cv2.cvtColor(cvImg, cv2.COLOR_RGB2BGR)
	
				Xob, Yob = SURF(queryImg,cvImg,surf)

				#cv2.imshow("Imagem NAO",cvImg)
	


	except KeyboardInterrupt:
		print
		print "Interrupted by user, shutting down"		  
	# relax !  no current in servos
	stiffnesses  = 0.0
	motionProxy.setStiffnesses(["Body"], stiffnesses)
	cameraProxy.unsubscribe(videoClient)
	sys.exit(0)

