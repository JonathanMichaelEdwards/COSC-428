#!python3.8

import math
import requests
import geopy.distance
from flask import Flask,render_template


API_KEY = 'TiWHj5tjCUzvT9TSgX2H2sCFyFuFjdd54Q-IGvmOzjk' # Acquire from 'developer.here.com'
 
# Defining latitude and longitude coordinates
_lat1 = '-43.52330968877337'     
_long1 = '172.5789769778771' 

_lat2 = '-43.523185686735815'   
_long2 = '172.5789048101865'  

print(f"Distance to Loaction = {geopy.distance.geodesic([_lat1, _long1] , [_lat2, _long2]).m} m")









# LICENSE: public domain - Jérôme Renard -- https://gist.github.com/jeromer -- jeromer@fastmail.net
# https://www.igismap.com/formula-to-find-bearing-or-heading-angle-between-two-points-latitude-longitude/

def calculateBearing(pointA, pointB):
    """
    - Calculates the bearing between two points.
    - TheThe formulae used is the following:
         θ = atan2(sin(Δlong).cos(lat2),
                   cos(lat1).sin(lat2) − sin(lat1).cos(lat2).cos(Δlong))

    Returns:
      The bearing in degrees
    Return Type:
      float
    """

    lat1 = math.radians(pointA[0])
    lat2 = math.radians(pointB[0])

    diffLong = math.radians(pointB[1] - pointA[1])

    x = math.sin(diffLong) * math.cos(lat2)
    y = math.cos(lat1) * math.sin(lat2) - (math.sin(lat1)
            * math.cos(lat2) * math.cos(diffLong))

    initial_bearing = math.atan2(x, y)

    # Now we have the initial bearing but math.atan2 return values
    # from -180° to + 180° which is not what we want for a compass bearing
    # The solution is to normalize the initial bearing as shown below
    initial_bearing = math.degrees(initial_bearing)
    compass_bearing = (initial_bearing + 360) % 360

    return compass_bearing









    
print(calculateBearing((float(_lat1), float(_long1)), (float(_lat2), float(_long2))))








#Flask code 
app = Flask(__name__)
@app.route('/')


def map_func():
	return render_template('map.html',apikey=API_KEY,lat1=_lat1,long1=_long1, lat2=_lat2,long2=_long2) # map.html is my HTML file name


if __name__ == '__main__':  
    app.run(debug = False)

