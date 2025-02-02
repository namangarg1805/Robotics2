#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Wed Mar  2 00:21:35 2022

@author: naman
"""

def calculate_D1_setpoints(rowno):
    row_y_cord=rowno*4-4
    rowcoordinate=(-3,row_y_cord,5)
    return rowcoordinate 
 
def calculate_D2_setpoints(rowno):
    #calculating setpoints of row
    if rowno==15:
        row_y_cord=4
        rowcoordinate=(0,-row_y_cord,3)
    else:
        row_y_cord=(15-rowno)*4+4
        rowcoordinate=(-3,-row_y_cord,3)
    return rowcoordinate 
    
rowd=[1,2,3,4,5,6,7,8,9,10,11,12,13,14,15]
x=list(map(calculate_D2_setpoints,rowd))
print(x)