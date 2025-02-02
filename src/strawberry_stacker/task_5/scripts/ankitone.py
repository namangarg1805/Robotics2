#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sun Jun 26 10:03:25 2022

@author: naman
"""



s=int(input())
e=int(input())
res=1
for i in range(s,e+1):
    if i%2==0:
        res=res*i
print(res)