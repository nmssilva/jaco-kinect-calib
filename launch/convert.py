import os, sys

launch = 'roslaunch export.launch '

print "Exporting frames from bagfile"
if (len(sys.argv) < 1):
	print "usage python convert.py <bagfile>"


launch += 'bag:="' + sys.argv[1] + '"'

print launch

os.system(launch)

print "Fetching frames from dump directory"
os.system('mkdir tmp')
os.system('mv ~/.ros/frame*.jpg tmp/')

nframes = len([name for name in os.listdir('tmp') if os.path.isfile('tmp/'+name) and name.startswith('frame')])

print "Fetched ", nframes, " frames."

concatcom = 'ffmpeg -i "concat:out00.mpg'

print "Joining frames into sample videos..."
for i in range(1,(nframes/100)+1):
	percent = i/(nframes/10000.0)
	print "Progress: " + "%03d" % percent + "%" 
	number = "%02d" % i
	command = 'convert -delay 6 -quality 100 tmp/frame'+number+'*jpg out'+number+'.mpg'
	os.system(command)
	concatcom += '|out'+number+'.mpg'

print "Progress: 100% DONE!\n Joining samples into final video..." 

concatcom += '" -c copy output.mpg'

print concatcom

os.system(concatcom)

