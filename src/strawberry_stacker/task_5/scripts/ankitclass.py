#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sun Jun 26 10:18:31 2022

@author: naman
"""

class Player:

    def __init__(self,pname,ptype,team,iscap,run,wicket):
        self.playerName=pname
        self.playerType=ptype
        self.team=team
        self.isCaptain=iscap
        self.runs=run
        self.wickets=wicket
    
    def CalculatePoints(self):
        if self.isCaptain=="True":
            fantasypoints=(self.runs+self.wickets*10)*2
        else:
            fantasypoints=self.runs+self.wickets*10
        
        return fantasypoints
    
class Tournament:
    def __init__(self,pllist):
        self.playerList=pllist
        
    def getPlayerList(self,ptype,points):
        rlist=[]
        for ourplayer in self.playerList:
            k=ourplayer.CalculatePoints()
            if ourplayer.playerType==ptype and k>points :
                rlist.append(ourplayer)
        if rlist==[]:
            return None
        return rlist                
    
    def findPlayerPoints(self,team,num):
        d={}
        for yourplayer in self.playerList:
            if yourplayer.team==team:
                k=yourplayer.CalculatePoints()
                if k>num:
                    d[yourplayer.playerName]=k
        if d:
            return d
        else:
            return None
        
noofplayer=int(input())
plist=[]
for i in range(noofplayer):
    pname=input()
    pteam=input()
    ptype=input()
    iscapt=input()
    runs=int(input())
    wickets=int(input())
    playerObject=Player(pname,ptype,pteam,iscapt,runs,wickets)
    plist.append(playerObject)
    
mytour=Tournament(plist)
mptype=input()
mpoints=int(input())
res1=mytour.getPlayerList(mptype,mpoints)
mteam=input()
npoints=int(input())
res2=mytour.findPlayerPoints(mteam,npoints)
if res1==None:
    print("Player Not Found")
else:
    for pl in res1:
        print(pl.playerName)
        print(pl.runs)
        print(pl.wickets)
    
if res2==None:
    print("Player Not Found")
else:
    for p,f in res2.items():
        print(p,f)++