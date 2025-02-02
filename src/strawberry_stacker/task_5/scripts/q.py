#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sun Jun 12 11:45:02 2022

@author: naman
"""
def mysolution(inp):
    st1=''
    st2=''
    st=''
    for i in inp:
        if i.islower():
            st1+=i
        else:
            st2+=i
    st=st1+st2
    if st=='':
        st="NULL"
    return st

no_of_ele=int(input())
li=[]
for i in range(no_of_ele):
    inp=input()
    li.append(inp)
for j in li:
    res=mysolution(j)
    print(res)


2*4*6*8*10



