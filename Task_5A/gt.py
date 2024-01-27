a, dx, dy = 50, 12, -14

angle = 0

dir = {-90:"up",
         0:"right",
        90:"down",
       180:"left"}

orientation = dir[angle]

if a>0 or a<0:
  if -10>a>-80 or 10<a<80 :
    agl = 1

  if -10<a<10 :
    agl = 2

  if -80>a or a>80 :
    agl = 3

print(agl)