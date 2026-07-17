# adaptive traffic light controller (atlc)
welcome to my atlc project! 

this project tries to fix a really annoying problem: traffic jams at intersections. 
normal traffic lights just use timers. even if a road is totally empty or super packed, the green light stays on for the exact same time. that's just dumb and inefficient.

so i built a smart traffic light system that actually looks at how many cars are waiting and changes the green light time on the fly. 

to do this, i used a fuzzy logic controller. basically, it mimics how a human would think (like, "if the line is super long, give it a long green light"). but to make it work really well, we trained it using some ai optimization algorithms: particle swarm optimization (pso) and ant colony optimization (aco).

here's what you'll find in this docs site:

1. how it works: a quick rundown of the fuzzy logic stuff.
2. optimization: how i used pso and aco to train the system.
3. performance results: the final tests showing it cuts wait times by like 50%.
4. code reference: some auto-generated docs if you want to look at the code.
