
/*
 * These functions are no longer a part of my code, but I am preserving theme here
 * for the sake of documentation, and for possible future use. 
 */

// ------------------------------ Global Variables -------------------------------

int n = 0;
int ne = 45;
int e = 90;
int se = 135;
int s = 180;
int sw = 225;
int w = 270;
int nw = 315;


// ---------------------------------- Functions ----------------------------------

/*
    FUNCTION: set_cardinal()
    ARGUMENTS: int currentHeading
    RETURN: None

    DESCRIPTION:
    Corrects the value of cardinal directions to account for starting
    heading.

    To obtain the heading offset, subtract the current heading or starting
    heading from 360. Then, to obtain the new value for a cardinal direction,
    add the offset to it's regular value. If the new value is greater than or
    equal to 360, subtract 360 to get the adjusted new value.

    The cardinal directions and their regular numeric value are as follows:
    North (n)       0
    Northeast (ne)  45
    East (e)        90
    Southeast (se)  135
    South (s)       180
    Southwest (sw)  225
    West (w)        270
    Northwest (nw)  315
*/
void set_cardinal()
{
  // variables
  int currentHeading = getCurrentHeading();          // starting direction
  int headingOffset = 360 - currentHeading;   // offset to add to cardinals

  // obtain new cardinal values
  n  = (n  + headingOffset) % 360;
  ne = (ne + headingOffset) % 360;
  e  = (e  + headingOffset) % 360;
  se = (se + headingOffset) % 360;
  s  = (s  + headingOffset) % 360;
  sw = (sw + headingOffset) % 360;
  w  = (w  + headingOffset) % 360;
  nw = (nw + headingOffset) % 360;
}

/*
    FUNCTION: subnav()
    ARGUMENTS: None
    RETURN: None

    DESCRIPTION:
    subnav() calculates a path to the next waypoint if there is one,
    if there is not one, it exits
*/
void subnav(double destination[])
{
  // variables
  double curr[2];

  getCurrentCoordinates(curr);

  while ((curr[0] < destination[0] - 0.000050 || 
          curr[0] > destination[0] + 0.000050) && 
         (curr[1] < destination[1] - 0.000050 ||
          curr[1] > destination[1] + 0.000050))
  {
    // current latitude is greater, longitude is greater
    while (curr[0] > destination[0] && curr[1] > destination[1])
    {
      drive(se, 40, 10);
      getCurrentCoordinates(curr);

      while (checkDist() == 1)
      {
        avoidObstacle();
      }
    }

    // current latitude is greater, longitude is lesser
    while (curr[0] > destination[0] && curr[1] < destination[1])
    {
      drive(ne, 40, 10);
      getCurrentCoordinates(curr);

      while (checkDist() == 1)
      {
        avoidObstacle();
      }
    }

    // current latitude is lesser, longitude is greater
    while (curr[0] < destination[0] && curr[1] > destination[1])
    {
      drive(sw, 40, 10);
      getCurrentCoordinates(curr);

      while (checkDist() == 1)
      {
        avoidObstacle();
      }
    }

    // current latitude is lesser, longitude is lesser
    while (curr[0] < destination[0] && curr[1] < destination[1])
    {
      drive(nw, 40, 10);
      getCurrentCoordinates(curr);

      while (checkDist() == 1)
      {
        avoidObstacle();
      }
    }

    // current latitude is greater, longitude is correct
    while (curr[0] > destination[0] && curr[1] == destination[1])
    {
      drive(e, 40, 10);
      getCurrentCoordinates(curr);

      while (checkDist() == 1)
      {
        avoidObstacle();
      }
    }

    // current latitude is lesser, longitude is correct
    while (curr[0] < destination[0] && curr[1] == destination[1])
    {
      drive(w, 40, 10);
      getCurrentCoordinates(curr);

      while (checkDist() == 1)
      {
        avoidObstacle();
      }
    }

    // current latitude is correct, longitude is greater
    while (curr[0] == destination[0] && curr[1] > destination[1])
    {
      drive(s, 40, 10);
      getCurrentCoordinates(curr);

      while (checkDist() == 1)
      {
        avoidObstacle();
      }
    }

    // current latitude is correct, longitude is lesser
    while (curr[0] == destination[0] && curr[1] < destination[1])
    {
      drive(n, 40, 10);
      getCurrentCoordinates(curr);

      while (checkDist() == 1)
      {
        avoidObstacle();
      }
    }
  }
}

/*
    FUNCTION: navigate()
    ARGUMENTS: None
    RETURN: None

    DESCRIPTION:
    navigate() directs the rover to navigate along the coordinates
    contained by the route[] array.

    We start at point start, then navigate to Waypoint[0]. Then we
    proceed to navigate from 0 to 1, 1 to 2, 2 to 3, 3 to 4, and then
    from 4 back to start.
*/
void navigate()
{
  for (int i = 0; i < 5; i++)
  {
    subnav(route[i]);  // navigate from where we are to this waypoint
    waypointProcedure();
  }

  subnav(start); // navigate back to the start
  waypointProcedure();
}

