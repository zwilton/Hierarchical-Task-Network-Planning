begin_version
3
end_version
begin_metric
0
end_metric
11
begin_variable
var0
-1
2
Atom clear(b1)
NegatedAtom clear(b1)
end_variable
begin_variable
var1
-1
2
Atom clear(b2)
NegatedAtom clear(b2)
end_variable
begin_variable
var2
-1
2
Atom clear(b3)
NegatedAtom clear(b3)
end_variable
begin_variable
var3
-1
2
Atom clear(b4)
NegatedAtom clear(b4)
end_variable
begin_variable
var4
-1
2
Atom clear(b5)
NegatedAtom clear(b5)
end_variable
begin_variable
var5
-1
2
Atom handempty()
NegatedAtom handempty()
end_variable
begin_variable
var6
-1
6
Atom holding(b1)
Atom on(b1, b2)
Atom on(b1, b3)
Atom on(b1, b4)
Atom on(b1, b5)
Atom ontable(b1)
end_variable
begin_variable
var7
-1
6
Atom holding(b2)
Atom on(b2, b1)
Atom on(b2, b3)
Atom on(b2, b4)
Atom on(b2, b5)
Atom ontable(b2)
end_variable
begin_variable
var8
-1
6
Atom holding(b3)
Atom on(b3, b1)
Atom on(b3, b2)
Atom on(b3, b4)
Atom on(b3, b5)
Atom ontable(b3)
end_variable
begin_variable
var9
-1
6
Atom holding(b4)
Atom on(b4, b1)
Atom on(b4, b2)
Atom on(b4, b3)
Atom on(b4, b5)
Atom ontable(b4)
end_variable
begin_variable
var10
-1
6
Atom holding(b5)
Atom on(b5, b1)
Atom on(b5, b2)
Atom on(b5, b3)
Atom on(b5, b4)
Atom ontable(b5)
end_variable
6
begin_mutex_group
6
0 0
6 0
7 1
8 1
9 1
10 1
end_mutex_group
begin_mutex_group
6
1 0
6 1
7 0
8 2
9 2
10 2
end_mutex_group
begin_mutex_group
6
2 0
6 2
7 2
8 0
9 3
10 3
end_mutex_group
begin_mutex_group
6
3 0
6 3
7 3
8 3
9 0
10 4
end_mutex_group
begin_mutex_group
6
4 0
6 4
7 4
8 4
9 4
10 0
end_mutex_group
begin_mutex_group
6
5 0
6 0
7 0
8 0
9 0
10 0
end_mutex_group
begin_state
1
0
1
1
0
0
2
5
5
1
4
end_state
begin_goal
5
6 5
7 5
8 5
9 1
10 2
end_goal
50
begin_operator
pick-up b1
0
3
0 0 0 1
0 5 0 1
0 6 5 0
1
end_operator
begin_operator
pick-up b2
0
3
0 1 0 1
0 5 0 1
0 7 5 0
1
end_operator
begin_operator
pick-up b3
0
3
0 2 0 1
0 5 0 1
0 8 5 0
1
end_operator
begin_operator
pick-up b4
0
3
0 3 0 1
0 5 0 1
0 9 5 0
1
end_operator
begin_operator
pick-up b5
0
3
0 4 0 1
0 5 0 1
0 10 5 0
1
end_operator
begin_operator
put-down b1
0
3
0 0 -1 0
0 5 -1 0
0 6 0 5
1
end_operator
begin_operator
put-down b2
0
3
0 1 -1 0
0 5 -1 0
0 7 0 5
1
end_operator
begin_operator
put-down b3
0
3
0 2 -1 0
0 5 -1 0
0 8 0 5
1
end_operator
begin_operator
put-down b4
0
3
0 3 -1 0
0 5 -1 0
0 9 0 5
1
end_operator
begin_operator
put-down b5
0
3
0 4 -1 0
0 5 -1 0
0 10 0 5
1
end_operator
begin_operator
stack b1 b2
0
4
0 0 -1 0
0 1 0 1
0 5 -1 0
0 6 0 1
1
end_operator
begin_operator
stack b1 b3
0
4
0 0 -1 0
0 2 0 1
0 5 -1 0
0 6 0 2
1
end_operator
begin_operator
stack b1 b4
0
4
0 0 -1 0
0 3 0 1
0 5 -1 0
0 6 0 3
1
end_operator
begin_operator
stack b1 b5
0
4
0 0 -1 0
0 4 0 1
0 5 -1 0
0 6 0 4
1
end_operator
begin_operator
stack b2 b1
0
4
0 0 0 1
0 1 -1 0
0 5 -1 0
0 7 0 1
1
end_operator
begin_operator
stack b2 b3
0
4
0 1 -1 0
0 2 0 1
0 5 -1 0
0 7 0 2
1
end_operator
begin_operator
stack b2 b4
0
4
0 1 -1 0
0 3 0 1
0 5 -1 0
0 7 0 3
1
end_operator
begin_operator
stack b2 b5
0
4
0 1 -1 0
0 4 0 1
0 5 -1 0
0 7 0 4
1
end_operator
begin_operator
stack b3 b1
0
4
0 0 0 1
0 2 -1 0
0 5 -1 0
0 8 0 1
1
end_operator
begin_operator
stack b3 b2
0
4
0 1 0 1
0 2 -1 0
0 5 -1 0
0 8 0 2
1
end_operator
begin_operator
stack b3 b4
0
4
0 2 -1 0
0 3 0 1
0 5 -1 0
0 8 0 3
1
end_operator
begin_operator
stack b3 b5
0
4
0 2 -1 0
0 4 0 1
0 5 -1 0
0 8 0 4
1
end_operator
begin_operator
stack b4 b1
0
4
0 0 0 1
0 3 -1 0
0 5 -1 0
0 9 0 1
1
end_operator
begin_operator
stack b4 b2
0
4
0 1 0 1
0 3 -1 0
0 5 -1 0
0 9 0 2
1
end_operator
begin_operator
stack b4 b3
0
4
0 2 0 1
0 3 -1 0
0 5 -1 0
0 9 0 3
1
end_operator
begin_operator
stack b4 b5
0
4
0 3 -1 0
0 4 0 1
0 5 -1 0
0 9 0 4
1
end_operator
begin_operator
stack b5 b1
0
4
0 0 0 1
0 4 -1 0
0 5 -1 0
0 10 0 1
1
end_operator
begin_operator
stack b5 b2
0
4
0 1 0 1
0 4 -1 0
0 5 -1 0
0 10 0 2
1
end_operator
begin_operator
stack b5 b3
0
4
0 2 0 1
0 4 -1 0
0 5 -1 0
0 10 0 3
1
end_operator
begin_operator
stack b5 b4
0
4
0 3 0 1
0 4 -1 0
0 5 -1 0
0 10 0 4
1
end_operator
begin_operator
unstack b1 b2
0
4
0 0 0 1
0 1 -1 0
0 5 0 1
0 6 1 0
1
end_operator
begin_operator
unstack b1 b3
0
4
0 0 0 1
0 2 -1 0
0 5 0 1
0 6 2 0
1
end_operator
begin_operator
unstack b1 b4
0
4
0 0 0 1
0 3 -1 0
0 5 0 1
0 6 3 0
1
end_operator
begin_operator
unstack b1 b5
0
4
0 0 0 1
0 4 -1 0
0 5 0 1
0 6 4 0
1
end_operator
begin_operator
unstack b2 b1
0
4
0 0 -1 0
0 1 0 1
0 5 0 1
0 7 1 0
1
end_operator
begin_operator
unstack b2 b3
0
4
0 1 0 1
0 2 -1 0
0 5 0 1
0 7 2 0
1
end_operator
begin_operator
unstack b2 b4
0
4
0 1 0 1
0 3 -1 0
0 5 0 1
0 7 3 0
1
end_operator
begin_operator
unstack b2 b5
0
4
0 1 0 1
0 4 -1 0
0 5 0 1
0 7 4 0
1
end_operator
begin_operator
unstack b3 b1
0
4
0 0 -1 0
0 2 0 1
0 5 0 1
0 8 1 0
1
end_operator
begin_operator
unstack b3 b2
0
4
0 1 -1 0
0 2 0 1
0 5 0 1
0 8 2 0
1
end_operator
begin_operator
unstack b3 b4
0
4
0 2 0 1
0 3 -1 0
0 5 0 1
0 8 3 0
1
end_operator
begin_operator
unstack b3 b5
0
4
0 2 0 1
0 4 -1 0
0 5 0 1
0 8 4 0
1
end_operator
begin_operator
unstack b4 b1
0
4
0 0 -1 0
0 3 0 1
0 5 0 1
0 9 1 0
1
end_operator
begin_operator
unstack b4 b2
0
4
0 1 -1 0
0 3 0 1
0 5 0 1
0 9 2 0
1
end_operator
begin_operator
unstack b4 b3
0
4
0 2 -1 0
0 3 0 1
0 5 0 1
0 9 3 0
1
end_operator
begin_operator
unstack b4 b5
0
4
0 3 0 1
0 4 -1 0
0 5 0 1
0 9 4 0
1
end_operator
begin_operator
unstack b5 b1
0
4
0 0 -1 0
0 4 0 1
0 5 0 1
0 10 1 0
1
end_operator
begin_operator
unstack b5 b2
0
4
0 1 -1 0
0 4 0 1
0 5 0 1
0 10 2 0
1
end_operator
begin_operator
unstack b5 b3
0
4
0 2 -1 0
0 4 0 1
0 5 0 1
0 10 3 0
1
end_operator
begin_operator
unstack b5 b4
0
4
0 3 -1 0
0 4 0 1
0 5 0 1
0 10 4 0
1
end_operator
0
