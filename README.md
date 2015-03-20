# ai-midterm

#Current Status
	-Detects starting location
	-Uses rays to map room
	-Parses rays and fills in knoweldge of room
	-Maintains list of explored/unexplored locations
	-Chooses farthest unexplored location as exploration goal
	-Travels using A* algorithm


#BUGS
	-The framework looks like it has a hard time dealing with lots of rays
		We probably only need to use 8, max

#TODO
	-Make smoother paths, i.e. stop less frequently
		This should be fixed by indiscretization, needs to be tested
	-Don't get stuck on corners
	-Actually pick up tofu
	-Make better decisions of where to travel to
	-Test indiscretized pathfinding

![screenshot](https://raw.githubusercontent.com/kmackenzieii/ai-midterm/master/Mapping.tiff)
