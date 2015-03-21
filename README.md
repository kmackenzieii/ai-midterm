# ai-midterm

#Current Status
	-Detects starting location
	-Uses rays to map room
	-Parses rays and fills in knoweldge of room
	-Maintains list of explored/unexplored locations
	-Chooses nearest unexplored location outside of its radius of sight
	-Travels using A* algorithm

![screenshot](https://raw.githubusercontent.com/kmackenzieii/ai-midterm/master/Mapping.tiff)

#BUGS
	-The framework looks like it has a hard time dealing with lots of rays
		We probably only need to use 8, max
	-Getting stuck in corners is still a big problem

#TODO
	-Make smoother paths, i.e. stop less frequently
		In order to do this, there needs to be some core restructuring. The current system requires frequent
		stopping to call rays
		
	-Don't get stuck on corners
	-Actually pick up tofu

