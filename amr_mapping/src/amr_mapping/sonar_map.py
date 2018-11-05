#!/usr/bin/env python

PACKAGE = 'amr_mapping'

import sys
import math
import rospy
from exceptions import Exception
from amr_mapping.map_store_py import MapStore, MapStoreCone

import random

class SonarMap:
    """
    Ported from C++ by Ivan Vishiakou
    Original description still applies:

    This class implements the sonar map algorithm as described in "Sonar-Based
    Real-World Mapping and Navigation" by Alberto Elfes, IEEE Journal Of
    Robotics And Automation, Vol. RA-3, No. 3, June 1987.
    
    Map coordinates vs. map cells
    
    The map works internaly on a discrete, integer-based grid, but exposes a
    more natural continuous coordinates interface. This allows an application
    to work with the map using its own units (in this documentation refered to
    as "meters"), without taking care of details of the map storage
    implementation.
    
    Each cell with integer coordinates (c_x, c_y) occupies the space from
    ((c_x - 0.5, c_y - 0.5) * resolution) exclusive to
    ((c_x - 0.5, c_y + 0.5) * resolution) inclusive.
    
    The value resolution is the length of a cells edge. All cells are considered
    to be squares.
    
    Note: if a variable name starts with the prefix "m_", then this variable
    contains a map coordinate. If the name starts with "c_" then this variable
    contains a cell coordinate. This convention applies both to the functions'
    arguments and internal/local variables.
    """

    
    def __init__(self, resolution, m_size_x, m_size_y):
        """
        This constructor creates a map of given dimensions.
        
        Args:
            resolution (float) : size of a cell, measured in meters, i.e. the length of
        the edge of a cell.
            m_size_x (float) : initial size of the map in x direction (meters).
            m_size_y (float) : initial size of the map in y direction (meters).
        """
        self._resolution = resolution
        self._c_size_x = int(round(m_size_x/resolution+2))|1
        self._c_size_y = int(round(m_size_y/resolution+2))|1
        self._m_size_x = resolution*self._c_size_x
        self._m_size_y = resolution*self._c_size_y
        self._m_min_x = -self._m_size_x/2.0
        self._m_min_y = -self._m_size_y/2.0
        
        self._map_combined = MapStore(self._c_size_x, self._c_size_y)
        self._map_free = MapStore(self._c_size_x, self._c_size_y)
        self._map_occupied = MapStore(self._c_size_x, self._c_size_y)
    
    
    def add_scan(self, m_sonar_x, m_sonar_y, sonar_theta, field_of_view,
                 max_range, registerd_range, uncertainty):
        """
        Update map using a sonar reading.

        Args:
            m_sonar_x (float) : x coordinate of the sonar in map coordinates.
            m_sonar_y (float) : y coordinate of the sonar in map coordinates.
            sonar_theta (float) : orientation of the sonar in map coordinates.
            fov (float) : opening angle of the sonar (radians).
            max_range (float) : maximum possible range of the sonar (meters).
            distance (float) : range reading returned from the sensor (meters).
            uncertainty (float) : the noise associated with the sensed distance,
        expressed as the standard deviation.
        """

        
        #============================== YOUR CODE HERE ==============================
        """
        Instructions: implement the routine that performs map update based on a
                      single sonar reading.

        Hint: use convertToCell() and convertToMap() functions to convert between
              map and cell coordinates.

        Hint: use MapStoreCone class to iterate over the cells covered
              by the sonar cone.

        Hint: you may use helper functions provided to you (euclidian_distance, 
              angular_distance, clamp)

        Some snippets for your convenience.
        MapStoreCone creating:

        cone = MapStoreCone(c_sonar_pos_x, c_sonar_pos_y, sonar_theta,
                            field_of_view, c_cone_length)
            the last argument is cone length in cells (since all the linear sizes it operates
            are in terms of cells!)

        Iterating over cone cells:
        for cell in cone:
            #Do stuff
            # cell is a tuple (c_x, c_y)
            pass

        Reading/Setting cells of the map (MapStore):

        occ_val = self._map_occupied.get(*cell)         # cell = (c_x, c_y)
        occ_val = self._map_occupied.get(c_x, c_y)      #

        self._map_occupied.set(c_x, c_y, new_occ_val)   # setting map cell value

        Hint: see the _convert_to_map(cell) function description. It can be used
              to check whether a cell is in bounds of the map. (You do not need to set/read
              cells that lie beyond the map grid)
              
              Alternatively you can use
              self._map_combined.is_in_x_range(c_x)     # returns boolean
              self._map_combined.is_in_y_range(c_y)     # returns boolean

        //============================================================================
        """
        
        
        cell_sonar_x, cell_sonar_y = ((float('inf'), float('inf')), self._convert_to_cell((m_sonar_x,m_sonar_y)))[self._convert_to_cell((m_sonar_x,m_sonar_y))!=None]

        # checking for valid cell coordinates
        if cell_sonar_x !=float('inf') and cell_sonar_y !=float('inf'):
            
            
            map_dist_x = m_sonar_x + registerd_range*math.cos(sonar_theta)
            map_dist_y = m_sonar_y + registerd_range*math.sin(sonar_theta)


            cell_dist_x, cell_dist_y = ((0,0), self._convert_to_cell((map_dist_x,map_dist_y)))[self._convert_to_cell((map_dist_x,map_dist_y))!=None]


            cell_distance = self.euclidian_distance(cell_sonar_x, cell_sonar_y, cell_dist_x, cell_dist_y)

            # probability variables
            p_free = 0.0
            p_occ = 0.0
            occupied_sum = 0.0  # normalization constant

            # Creating an object for the sonar cone
            cone = MapStoreCone(cell_sonar_x, cell_sonar_y, sonar_theta, field_of_view, cell_distance)

            for cell in cone:

                map_x, map_y = ((float('inf'), float('inf')), self._convert_to_map(cell))[self._convert_to_map(cell)!=None]

                # checking for cell coordinates within the map
                if map_x!=float('inf') and map_y!=float('inf'):
                   
                    theta = self.angular_distance(math.atan2((map_y - m_sonar_y) , (map_x - m_sonar_x)), sonar_theta)
                    delta = self.euclidian_distance(m_sonar_x, m_sonar_y, map_x, map_y)

                    er_free = self._er_free(registerd_range, delta, uncertainty)
                    ea = self._ea(field_of_view,theta)
                    er_occ = self._er_occ(registerd_range, delta, uncertainty)

                   # Calculating the prob of free space within sonar reading range
                    if(delta >= 0 and delta < registerd_range - uncertainty):
                        p_free = er_free * ea

                       # Calculating prob of free space
                        p_free += self._map_free.get(cell[0], cell[1]) - self._map_free.get(cell[0], cell[1]) * p_free

                       # Clipping the probabilities to [-1,1] range
                        p_free = self.clamp(p_free, -1, 1)

                        # Draw map
                        self._map_free.set(cell[0],cell[1], p_free)


                    elif (delta>=registerd_range - uncertainty and delta < registerd_range + uncertainty and registerd_range != max_range):

                        # Calculating prob of occupied space
                        p_occ = er_occ * ea
                        
                        p_occ *= (1 - self._map_free.get(cell[0], cell[1]))
                        
                        occupied_sum += p_occ
                        
                        if occupied_sum != 0:

                            p_occ /= occupied_sum
                            
                            p_occ += self._map_occupied.get(cell[0], cell[1]) - self._map_occupied.get(cell[0],cell[1])*p_occ

                            # Clipping the probabilities to [-1,1] range
                            p_occ = self.clamp(p_occ, -1,1)
                            
                            # Draw map
                            self._map_occupied.set(cell[0],cell[1], p_occ)

                        # Draw map
                    if self._map_occupied.get(cell[0], cell[1]) >= self._map_free.get(cell[0], cell[1]) :
                        self._map_combined.set(cell[0],cell[1], self._map_occupied.get(cell[0], cell[1]))
                    else:
                        self._map_combined.set(cell[0],cell[1], -self._map_free.get(cell[0], cell[1]))

                       
    
    
    def _er_free(self, sensed_distance, delta, uncertainty):
        """
        Calculate free-space probability.
        
        This function calculates the probability to be free for a point that is
        delta meters away from the sonar's origin when the sonar has measured a
        distance of sensed_distance with given uncertainty. This function only
        computes the radial component of the probability. To fully specify
        a point you need a distance and an angle and as such for the full
        probability you need the angular probability of the point to be the cause
        of the measured sensed_distance. This is calculated by _ea().
        The full probability is the product of the result from _ea() and from this
        function.
        
        Args:
            sensed_distance (float) : distance in meters measured by the sonar.
            delta (float) : distance from the sonar's origin for which the probability
        should be calculated.
            uncertainty (float) : uncertainty (variance) of measured distance.
        Returns:
            float : The probability to be free for a point delta meters away from
        the sonar's origin. The value is in the range 0.0 to 1.0.
        """
        
        #============================== YOUR CODE HERE ==============================
        #Instructions: compute the distance probability function for the "probably
        #              empty" region.
        
        prob = 0.0
        if delta >= 0 and delta< (sensed_distance - uncertainty) :
            prob = 1- pow((delta)/(sensed_distance-uncertainty),2)


        return prob

    
    def _er_occ(self, sensed_distance, delta, uncertainty):
        """
        Calculate occupied-space probability.
        
        This function calculates the probability to be occupied for a point that
        is delta meters away from the sonar's origin when the sonar has
        measured a distance of sensed_distance with an uncertainty of
        uncertainty. This function only computes the radial component of
        the probability. To fully specify a point you need a distance and an angle
        and as such for the full probability you need the angular probability of
        the point to be the cause of the measured sensed_distance.
        This is calculated by _ea(). The full probability is the product of the
        result from _ea() and from this function.
        
        Args:
            sensed_distance (float) : distance in meters measured by the sonar.
            delta (float) : distance from the sonar's origin for which the probability
        should be calculated.
            uncertainty (float) : uncertainty (variance) of measured distance.
        
        Returns:
            float : The probability to be occupied for a point delta meters away
        from the sonar's origin. The value is in the range 0.0 to 1.0.
        """

        #============================== YOUR CODE HERE ==============================
        #Instructions: compute the distance probability function for the "probably
        #              occupied" region.
        prob = 0.0
        if delta>=(sensed_distance - uncertainty) and delta < (sensed_distance + uncertainty) :
            prob = 1 - pow((delta-sensed_distance)/uncertainty, 2)


        return prob
    
    
    def _ea(self, sonar_fov, theta):
        """
        Probability for a point in the sonar cone to be actually measured.

        This function calculates the probability of a point theta radians away
        from the center beam of a sonar cone of sonar_fov angular width, to be
        the cause of a sonar measurement.
        
        Args:
            sonar_fov (float) : the opening angle of the sonar cone in radians.
            theta (float) : the angular distance of a point from the center of the
        sonar cone, measured in radians. This value must lie within plus/minus
        sonar_fov / 2.

        Returns:
            float : Probability of a point in the sonar cone to be measured, 
        value in range 0.0 to 1.0 as a funciton of angular distance to the
        central line of the sonar cone.
        """

        #============================== YOUR CODE HERE ==============================
        #Instructions: compute the angular probability function (it is same for both
        #              the "probably empty" and the "probably occupied" region.
        prob = 0.0
        if theta>-sonar_fov/2 and theta < sonar_fov/2:
            prob = 1.0 - pow((2*theta)/sonar_fov, 2)
           

        return prob
    
    
    def _convert_to_map(self, c_pos):
        """
        Converts cell coordinates to metric coordinates. Examples:
        m_pos = convert_to_map(c_pos)       #c_pos = (c_x, c_y)
        
        returns tuple (m_x, m_y) or None if cell coordinates out of bounds
        """
        c_x, c_y = c_pos
        if ( self._map_combined.is_in_x_range(c_x) and
             self._map_combined.is_in_y_range(c_y)      ):
            return (c_x*self._resolution, c_y*self._resolution)
        else:
            return None
    
    
    def _convert_to_cell(self, m_pos):
        """
        Converts metric coordinates cell coordinates. Examples:
        c_pos = convert_to_map(m_pos)       #m_pos = (m_x, m_y)
        
        returns tuple (c_x, c_y) or None if cell coordinates out of bounds
        """
        m_x, m_y = m_pos
        c_x, c_y = int(round(m_x / self._resolution)), int(round(m_y / self._resolution))
        if ( self._map_combined.is_in_x_range(c_x) and
             self._map_combined.is_in_y_range(c_y)      ):
            return (c_x, c_y)
        else:
            return None


    def get_grid_size_x(self):
        return self._c_size_x
    
    def get_grid_size_y(self):
        return self._c_size_y
    
    
    def get_min_x(self):
        return self._m_min_x
        
    def get_min_y(self):
        return self._m_min_y
    
    
    def get_resolution(self):
        return self._resolution
    
    def get_map_data(self):
        return self._map_combined.get_publish_data(-1.0, 1.0)
    
    def get_map_free_data(self):
        return self._map_free.get_publish_data(0.0, 1.0)
        
    def get_map_occupied_data(self):
        return self._map_occupied.get_publish_data(0.0, 1.0)
    
    
    @staticmethod
    def euclidian_distance(*args):
        """ Returns euclidian distance between two points:
        dist = euclidian_distance(pos1, pos2)
        dist = euclidian_distance(x1, y1, x2, y2)
        """
        try:
            if len(args) == 2:
                #tuples
                return math.sqrt((args[0][0]-args[1][0])**2+(args[0][1]-args[1][1])**2)
            elif len(args) == 4:
                #x's and y's:
                return math.sqrt((args[0]-args[2])**2+(args[1]-args[3])**2)
            else:
                raise Exception('Invalid arguments for euclidian_distance()')
        except Exception as e:
            print "Exception caught:", e.message
    
    
    @staticmethod
    def angular_distance(a1, a2):
        """Returns angular distance between two angles"""
        return math.atan2(math.sin(a1-a2), math.cos(a1-a2))
    
    
    @staticmethod
    def clamp(value, min_val, max_val):
        """Helper function to clamp a variable to a given range."""
        return max(min(value, max_val), min_val)