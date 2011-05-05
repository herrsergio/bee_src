
/***************************************************************************
 ***************************************************************************
 ***************************************************************************
 *****
 ***** Welcome!
 *****
 ***** This file is part of the BeeSoft robot control software.
 ***** The version number is:
 *****
 *****                  v1.3.8 (released Aug 5, 1998)
 *****                  this is not an official BeeSoft version
 *****
 ***** Please refer to bee/src/COPYRIGHT for copyright and liability
 ***** information.
 *****
 ***************************************************************************
 ***************************************************************************
 ***************************************************************************/





/* CDtcx CD-ROM Player-Software
   '96 by Arno Buecken
   
   based on 
   
   WorkBone CD Rom Player Software
    
   Copyright (c) 1994  Thomas McWilliams
       
   This program is free software; you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation; either version 2, or (at your option)
   any later version.
                   
   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.
                               
   You should have received a copy of the GNU General Public License
   along with this program; if not, write to the Free Software
   Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
                                        
*/
                                         



#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "struct.h"
#include "workbone.h"



#ifdef UNIBONN

CD_msg_type MSGs[NUM_MSGs] = {
/* 0 */  { 1, 0, 0, 1, 1, 0, "GONG" },
/* 1 */  { 1, 3, 0, 1, 4, 0, "Xylophon"},
/* 2 */  { 1, 7, 0, 1, 8, 0, "Windows-Tata"},
/* 3 */  { 1,10, 0, 2, 0, 0, "Akkord"},
/* 4 */  { 2, 0, 0, 2, 3, 0, "Cromwell-Fanfare"},
/* 5 */  { 2, 2, 0, 2, 7, 0, "Don't loose my number"},
/* 6 */  { 2, 8, 0, 2, 9, 0, "Wau"},
/* 7 */  { 2, 7, 0, 2, 9, 0, "WauWau"},
/* 8 */  { 2, 9, 0, 2,18, 0, "Groove"},
/* 9 */  { 2,18, 0, 2,25, 0, "We don't need no education"},
/* 10 */  { 2,25, 0, 2,27, 0, "I quit!"},
/* 11 */  { 2,30, 0, 2,32, 0, "Lok"},
/* 12 */  { 2,33, 0, 2,34, 0, "Ruelps"},
/* 13 */  { 2,34, 0, 2,39, 0, "Lachen"},
/* 14 */  { 2,40, 0, 2,44, 0, "Lachen 2"},
/* 15 */  { 2,45, 0, 2,46, 0, "Pfeife"},
/* 16 */  { 2,50, 0, 2,52, 0, "Star-Wars"},
/* 17 */  { 3, 0, 0, 3, 9, 0, "Hallo, mein Name ist Rhino. Ich bin Ihr persoenlicher Diener"},
/* 18 */  { 3, 6, 0, 3, 9, 0, "Ich werde Ihnen jetzt folgen"},
/* 19 */  { 3,11, 0, 3,13, 0, "Ich werde jetzt Ihr Buero aufraeumen"},
/* 20 */  { 3,13, 0, 3,18, 0, "Ich werde jetzt das Studio aufraeumen"},
/* 21 */  { 3,18, 0, 3,27, 0, "Vielen Dank, Sie haben einen einfachen Roboter sehr gluecklich gemacht"},
/* 22 */  { 3,27, 0, 3,35, 0, "Guten Abend meine sehr verehrten Damen und Herren"},
/* 23 */  { 3,35, 0, 3,41, 0, "Objekt der Kategorie Abfall detektiert"},
/* 24 */  { 3,41, 0, 3,47, 0, "Berechne Entfernung fuer genaue Annaeherung"},
/* 25 */  { 3,47, 0, 3,52, 0, "Objekt wird eingesammelt"},
/* 26 */  { 3,52, 0, 3,58, 0, "Objekt der Kategorie Muelleimer detektiert"},
/* 27 */  { 3,58, 0, 3,62, 0, "Objekt wird entsorgt"},
/* 28 */  { 3,62, 0, 3,68, 0, "Suche Objekte der Kategorie Abfall"},
/* 29 */  { 3,68, 0, 3,73, 0, "Suche Objekt der Kategorie Muelleimer"},
/* 30 */  { 3,73, 0, 3,80, 0, "Person im Blickfeld detektiert. Starte Verfolgungsmodus"},
/* 31 */  { 3,80, 0, 3,84, 0, "Person aus Blickfeld verloren"},
/* 32 */  { 3,84, 0, 3,91, 0, "Person laeuft nach rechts aus dem Blickfeld. Korrigiere Richtung"},
/* 33 */  { 3,91, 0, 3,97, 0, "Person laeuft nach links aus dem Blickfeld. Korrigiere Richtung"},
/* 34 */  { 3,97, 0, 3,101,0, "Neuer Zielpunkt empfangen"},
/* 35 */  { 3,101,0, 3,106,0, "Ich habe meine Aufgabe erfuellt"},
/* 36 */  { 3,106,0, 3,110,0, "Wandorientierung der aktuellen Umgebung erkannt"},
/* 37 */  { 3,110,0, 4, 0, 0, "Wechsle in Explorationsmodus"},
/* 38 */  { 4, 1, 0, 4, 3, 0, "Ich habe ein Objekt gefunden"},
/* 39 */  { 4, 5, 0, 4, 7, 0, "Ich fahre zum Objekt"},
/* 40 */  { 4, 8, 0, 4,11, 0, "Ich hebe das Objekt auf"},
/* 41 */  { 4,12, 0, 4,15, 0, "Ich habe einen Abfalleimer gefunden"},
/* 42 */  { 4,16, 0, 4,19, 0, "Ich fahre zum Abfalleimer"},
/* 43 */  { 4,20, 0, 4,24, 0, "Ich lasse das Objekt in den Abfalleimer fallen"},
/* 44 */  { 4,26, 0, 4,27, 0, "Huch?"},
/* 45 */  { 4,27, 0, 4,31, 0, "Ich stecke fest"},
/* 46 */  { 4,31, 0, 4,35, 0, "Hui, jetzt wirds schnell"},
/* 47 */  { 4,40, 0, 4,44, 0, "Vorsicht, der Arm ist draussen"},
/* 48 */  { 4,44, 0, 5, 0, 0, "Ich fahre ab sofort rueckwaerts"},
/* 49 */  { 5, 0, 0, 5, 6, 0, "Darf ich mich vorstellen, ich heisse Rhino, bin Ihr neuer Diener und folge Ihnen ueberall hin"},
/* 50 */  { 5, 6, 0, 5,11, 0, "Wie sieht es denn hier aus"},
/* 51 */  { 5,11, 0, 5,18, 0, "Sie spielen Tennis - und ich lach mich hier tot..."},
/* 52 */  { 5,18, 0, 6, 0, 0, "Sie spielen Tennis und ich bin Ihr neuer Balljunge"},
/* 53 */  { 6, 0, 0, 6, 3, 0, "Hi, ich bin Rhino"},
/* 54 */  { 6, 3, 0, 6,11, 0, "Mein Name ist Rhino - Rhino-Vitorio, ich weiche nicht von Deiner Seite"},
/* 55 */  { 6,11, 0, 6,15, 0, "Kannst Du nicht mal Ordnung halten"},
/* 56 */  { 6,15, 0, 6,20, 0, "Hey, hier stimmt was nicht"},
/* 57 */  { 6,20, 0, 6,25, 0, "Bringt mir sofort meinen Muelleimer zurueck"},
/* 58 */  { 6,25, 0, 6,31, 0, "Computer, gehe in Verfolgungsmodus"},
/* 59 */  { 6,31, 0, 6,36, 0, "Computer, Gebiet scannen"},
/* 60 */  { 6,36, 0, 6,41, 0, "rot/gruen gefunden"},
/* 61 */  { 6,41, 0, 6,46, 0, "Hey ich hab Dich verloren"},
/* 62 */  { 6,46, 0, 6,50, 0, "Muell in Sicht"},
/* 63 */  { 6,50, 0, 6,53, 0, "Schalte auf Warp-Antrieb"},
/* 64 */  { 6,53, 0, 6,56, 0, "Warp 9"},
/* 65 */  { 6,56, 0, 6,60, 0, "Volle Kraft zurueck"},
/* 66 */  { 6,60, 0, 7, 0, 0, "Holt mich hier raus"},
/* 67 */  { 7, 0, 0, 7, 3, 0, "Hi there!"},
/* 68 */  { 7, 3, 0, 7, 6, 0, "Arm is ready"},
/* 69 */  { 7, 6, 0, 7, 9, 0, "Base is ready"},
/* 70 */  { 7, 9, 0, 7,12, 0, "Vision is ready"},
/* 71 */  { 7,12, 0, 7,17, 0, "This is Rhino using precompiled Speech"},
/* 72 */  { 7,17, 0, 7,20, 0, "I see some trash"},
/* 73 */  { 7,20, 0, 7,24, 0, "I see a trash-bin"},
/* 74 */  { 7,24, 0, 7,26, 0, "How far is it away"},
/* 75 */  { 7,26, 0, 7,31, 0, "Gotcha"},
/* 76 */  { 7,31, 0, 8, 0, 0, "Doing Hyperjump!"},
/* 77 */  { 8, 0, 0, 8, 3, 0, "Tach auch ich bin de Rhino"},
/* 78 */  { 8, 5, 0, 8, 7, 0, "Wat soll denn de Driss he"},
/* 79 */  { 8, 9, 0, 8,12, 0, "Wo Sie jrad sagen Muell"},
/* 80 */  { 8,13, 0, 8,16, 0, "Zement ens, ich hann keen Brill"},
/* 81 */  { 8,16, 0, 9, 0, 0, "Kenn mer nich, bruch mer nich, fott damit"},
/* 82 */  { 9, 0, 0,10, 0, 0, "SONG: Happy birthday (short version)"},
/* 83 */  {10, 0, 0,11, 0, 0, "SONG: Happy birthday (long version)"},
/* 84 */  {12, 0, 0, 12, 4, 0, "Hello my name is Rhino"},
/* 85 */  {12, 4, 0, 12, 9, 0, "Could I offer you a tour through the building"},
/* 86 */  {12, 9, 0, 12, 12, 0, "This is the classroom"},
/* 87 */  {12, 12, 0, 12, 16, 0, "The classroom is used to teach courses"},
/* 88 */  {12, 16, 0, 12, 21, 0, "It is a large room and it is used frequently"},
/* 89 */  {12, 21, 0, 12, 27, 0, "This is our library, it is also the office of Erika Hoelzer"},
/* 90 */  {12, 27, 0, 12, 33, 0, "We have a total of 475.4 books in our libraray"},
/* 91 */  {12, 33, 0, 12, 38, 0, "There is also a laser-printer"},
/* 92 */  {12, 38, 0, 12, 43, 0, "This is the office of Sebastin Thrun and my home"},
/* 93 */  {12, 43, 0, 12, 49, 0, "Sebastian is a wonderful guy, you should meet him"},
/* 94 */  {12, 49, 0, 12, 58, 0, "This is the office of Dieter Fox, ..."},
/* 95 */  {12, 58, 0, 12, 62, 0, "You would be surprised who many people fit to one room"},

/* 96 */  {13, 0, 0, 13, 4, 0, "Press the red button"},
/* 97 */  {13, 4, 0, 13, 7, 0, "Press the yellow button"},
/* 98 */  {13, 7, 0, 13, 11, 0, "Press the green button"},
/* 99 */  {13, 11, 0, 13, 15, 0, "Press the blue button"},
/* 100 */  {13, 15, 0, 13, 22, 0, "Press the red if you wanna have more information"},
/* 101 */  {13, 22, 0, 13, 27, 0, "Press the yellow button if you wanna continue the tour"},
/* 102 */  {13, 27, 0, 13, 35, 0, "press the blue button if you wanna finish the tour"},
/* 103 */  {13, 35, 0, 13, 42, 0, "press the blue button if you're done with me"},
/* 104 */  {13, 42, 0, 13, 49, 0, "press the flashing button if you wanna terminate the tour"},
/* 105 */  {13, 49, 0, 13, 55, 0, "press the red button if you wanna continue the tour"},
/* 106 */  {13, 55, 0, 13, 62, 0, "press the green button if you wanna have more information"},
/* 107 */  {13, 62, 0, 13, 70, 0, "press the yellow if you want to have 10000 dollars"},
/* 108 */  {13, 70, 0, 13, 74, 0, "my battery level is low"},
/* 109 */  {13, 74, 0, 13, 78, 0, "you need more information"},
/* 110 */  {13, 78, 0, 13, 82, 0, "get out of my way"},
/* 111 */  {13, 82, 0, 13, 87, 0, "please get out of my way"},
/* 112 */  {13, 87, 0, 13, 91, 0, "I can't reach my destination"},
/* 113 */  {13, 91, 0, 13, 94, 0, "this door is closed"},
/* 114 */  {13, 94, 0, 13, 98, 0, "this door is open"},
/* 115 */  {13, 98, 0, 13, 101, 0, "I cannot see the door"},
/* 116 */  {13, 101, 0, 13, 105, 0, "i cannot see the door-label"},
/* 117 */  {13, 105, 0, 13, 109, 0, "i cannot read the door-label"},
/* 118 */  {13, 109, 0, 13, 112, 0, "i can see the door-label"},
/* 119 */  {13, 112, 0, 13, 115, 0, "i see the door"},
/* 120 */  {13, 115, 0, 13, 120, 0, "let me read the door-label for you"},

/* 121 */  {14, 0, 0, 14, 7, 0, "Hello my name is Rhino, are you interested in a tour today"},
/* 122 */  {14, 7, 0, 14, 10, 0, "This is the classroom"},
/* 123 */  {14, 10, 0, 14, 13, 0, "The classroom is used heavily"},
/* 124 */  {14, 13, 0, 14, 18, 0, "He has about 30 chairs and a huge blackboard"},
/* 125 */  {14, 18, 0, 14, 27, 0, "It is used heavily throughout the semester ..."},
/* 126 */  {14, 27, 0, 14, 31, 0, "Let's go to the next room"},
/* 127 */  {14, 31, 0, 14, 34, 0, "Get out of my way"},
/* 128 */  {14, 34, 0, 14, 40, 0, "This is our library. It is also the office of Erika Hoelzer"},
/* 129 */  {14, 40, 0, 14, 45, 0, "We have a total of about ...."},
/* 130 */  {14, 45, 0, 14, 50, 0, "The printer is also located in this library"},
/* 131 */  {14, 50, 0, 14, 53, 0, "This is the office of Sebastian Thrun"},
/* 132 */  {14, 53, 0, 14, 59, 0, "Sebastian is my master, he built most of my software"},
/* 133 */  {14, 59, 0, 14, 66, 0, "Sebastian is currently teaching at Carnegie Mellon ..."},
/* 134 */  {14, 66, 0, 14, 70, 0, "Sebastian is a young blonde guy"},
/* 135 */  {14, 70, 0, 14, 73, 0, "This is also my home"},
/* 136 */  {14, 73, 0, 14, 80, 0, "This is my office, and my slave Sebastian Thrun lives here as well"},
/* 137 */  {14, 80, 0, 14, 87, 0, "This is the office of myself and my assistant Sebastian Thrun"},
/* 138 */  {14, 87, 0, 14, 94, 0, "This is the office of Dieter Fox,..."},
/* 139 */  {14, 94, 0, 14, 98, 0, "Dieter Fox is one of my masters"},
/* 140 */  {14, 98, 0, 14, 104, 0, "Dieter is responsible for my Colli..."},

/* 141 */  {15, 0, 0, 15, 4, 0, "This is our classroom"},
/* 142 */  {15, 4, 0, 15, 7, 0, "The classroom is used for teaching"},
/* 143 */  {15, 7, 0, 15, 15, 0, "It has about 30 chairs, fits 40 people, ..."},
/* 144 */  {15, 15, 0, 15, 24, 0, "The classroom is used heavily throughout the semester ..."},
/* 145 */  {15, 24, 0, 15, 30, 0, "This is the office of Martin Breunig,..."},
/* 146 */  {15, 30, 0, 15, 38, 0, "Oleg is a scientist from the soviet union"},
/* 147 */  {15, 38, 0, 15, 50, 0, "I have no idea what these guys are working on, but ..."},
/* 148 */  {15, 50, 0, 15, 54, 0, "This is the office of Prof. Buhmann"},
/* 149 */  {15, 54, 0, 15, 60, 0, "Prof Buhmann is a young Prof in our department"},
/* 150 */  {15, 60, 0, 15, 67, 0, "He is concerned with ..."},
/* 151 */  {15, 67, 0, 15, 73, 0, "Prof Buhmann is responsible for some of my funding"},
/* 152 */  {15, 73, 0, 15, 78, 0, "I am very grateful to Prof. Buhmann"},

/* 153 */  {16, 0, 0, 16, 8, 0, "This is the office of Andreas Kusserow, ..."},
/* 154 */  {16, 8, 0, 16, 16, 0, "Chandra Reddy is a sientist from India"},
/* 155 */  {16, 16, 0, 16, 23, 0, "This is the office of Thorsten Froehlinghaus, ..."},
/* 156 */  {16, 23, 0, 16, 28, 0, "All three of these are working with J. Buhmann"},
/* 157 */  {16, 28, 0, 16, 35, 0, "They're working on robot control ..."},
/* 158 */  {16, 35, 0, 16, 40, 0, "This is the office of Prof. Dieter Fellner"},
/* 159 */  {16, 40, 0, 16, 50, 0, "Dieter Fellner is a young Prof, who is concerned ..."},
/* 160 */  {16, 50, 0, 16, 56, 0, "Excuse me, excuse me, are you interested in a tour"},
/* 161 */  {16, 56, 0, 16, 66, 0, "Excuse me, excuse me, my name is Rhino, are you interested ..."},
/* 162 */  {16, 66, 0, 16, 69, 0, "Excuse me"},
/* 163 */  {16, 69, 0, 16, 76, 0, "My name is Rhino, could I offer you a tour through the ..."},
/* 164 */  {16, 76, 0, 16, 84, 0, "This is Computer Science Department 3, its chair is ABC"},
/* 165 */  {16, 84, 0, 16, 91, 0, "The office of Armin Cremers is not in this corridor"},
/* 166 */  {16, 91, 0, 16, 98, 0, "The office of Professor Cremers is located elsewhere"},

/* 167 */  {17, 0, 0, 17, 4, 0, "Let's go to the next room"},
/* 168 */  {17, 4, 0, 17, 8, 0, "Let's go to the next object"},
/* 169 */  {17, 8, 0, 17, 10, 0, "Yes"},
/* 170 */  {17, 10, 0, 17, 13, 0, "No"},
/* 171 */  {17, 13, 0, 17, 23, 0, "Press thred button in the next 5 seconds, 4 3 2 1 Your time is over"},
/* 172 */  {17, 23, 0, 17, 27, 0, "I hope you enjoyed the tour"},
/* 173 */  {17, 27, 0, 17, 33, 0, "This is the end of the tour, I hope you enjoyed the tour"},
/* 174 */  {17, 33, 0, 17, 48, 0, "My name is Rhino I am a fully autonomous tour guide ..."},
/* 175 */  {17, 48, 0, 17, 56, 0, "If you like the tour donate 10 DM to Sebastian Thrun"},
/* 176 */  {17, 56, 0, 17, 60, 0, "This is room 121"},
/* 177 */  {17, 60, 0, 17, 65, 0, "This is room 120"},
/* 178 */  {17, 65, 0, 17, 70, 0, "This is room 111"},
/* 179 */  {17, 70, 0, 17, 75, 0, "This is room 120"},
/* 180 */  {17, 75, 0, 17, 79, 0, "This is room 119"},
/* 181 */  {17, 79, 0, 17, 84, 0, "This is room 112"},
/* 182 */  {17, 84, 0, 17, 88, 0, "This is room 118"},
/* 183 */  {17, 88, 0, 17, 92, 0, "This is room 114"},
/* 184 */  {17, 92, 0, 17, 96, 0, "This is room 117"},
/* 185 */  {17, 96, 0, 17, 100, 0, "There are more offices behind the glass-door"},
/* 186 */  {17, 100, 0, 17, 103, 0, "The door is closed"},
/* 187 */  {17, 103, 0, 17, 106, 0, "The door is open"},
/* 188 */  {17, 106, 0, 17, 109, 0, "I can't see you"},
/* 189 */  {17, 109, 0, 17, 113, 0, "Where are you"},
/* 190 */  {17, 113, 0, 17, 119, 0, "Press the red button to indicate that you are still interested ..."},

/* 191 */  {18, 0, 0, 18, 3, 0, "My sonar sensors are not working"},
/* 192 */  {18, 3, 0, 18, 6, 0, "My camera isn't working"},
/* 193 */  {18, 6, 0, 18, 11, 0, "My infrared sensors are not working properly"},
/* 194 */  {18, 11, 0, 18, 16, 0, "My tactile sensors are not working properly"},
/* 195 */  {18, 16, 0, 18, 20, 0, "My arm is deployed"},
/* 196 */  {18, 20, 0, 18, 22, 0, "Map is up"},
/* 197 */  {18, 22, 0, 18, 25, 0, "Arm is up"},
/* 198 */  {18, 25, 0, 18, 27, 0, "Map is ready"},
/* 199 */  {18, 27, 0, 18, 30, 0, "Arm is ready"},
/* 200 */  {18, 30, 0, 18, 33, 0, "Base is ready"},
/* 201 */  {18, 33, 0, 18, 36, 0, "Buttons is ready"},
/* 202 */  {18, 36, 0, 18, 39, 0, "Controller is ready"},
/* 203 */  {18, 39, 0, 18, 42, 0, "Pantilt is ready"},
/* 204 */  {18, 42, 0, 18, 45, 0, "Plan is ready"},
/* 205 */  {18, 45, 0, 18, 48, 0, "Simulator is ready"},
/* 206 */  {18, 48, 0, 18, 51, 0, "TCX is ready"},
/* 207 */  {18, 51, 0, 18, 54, 0, "Sonarint is ready"},
/* 208 */  {18, 54, 0, 18, 58, 0, "Stereovision is ready"},
/* 209 */  {18, 58, 0, 18, 63, 0, "Position-estimation is ready"},
/* 210 */  {18, 63, 0, 18, 67, 0, "This is my high-speed"},
/* 211 */  {18, 67, 0, 18, 70, 0, "This is as fast as I can move"},

/* 212 */  {19, 0, 0, 19, 3, 0, "Welcome to the tour"},
/* 213 */  {19, 3, 0, 19, 7, 0, "This is the beginning of the tour"},
/* 214 */  {19, 7, 0, 19, 15, 0, "You can now teach me positions, they will become part of the tour"},
/* 215 */  {19, 15, 0, 19, 19, 0, "Press the red button to teach me a position"},
/* 216 */  {19, 19, 0, 19, 24, 0, "Press the blue button to finish the tour"},
/* 217 */  {19, 24, 0, 19, 27, 0, "I will memorize this position"},
/* 218 */  {19, 27, 0, 19, 33, 0, "I will memorize the object in front of me"},
/* 219 */  {19, 33, 0, 19, 36, 0, "1"},
/* 220 */  {19, 36, 0, 19, 39, 0, "2"},
/* 221 */  {19, 39, 0, 19, 43, 0, "3"},
/* 222 */  {19, 43, 0, 19, 46, 0, "4"},
/* 223 */  {19, 46, 0, 19, 50, 0, "5"},
/* 224 */  {19, 50, 0, 19, 53, 0, "6"},
/* 225 */  {19, 53, 0, 19, 57, 0, "7"},
/* 226 */  {19, 57, 0, 19, 60, 0, "8"},
/* 227 */  {19, 60, 0, 19, 64, 0, "9"},
/* 228 */  {19, 64, 0, 19, 67, 0, "10"},
/* 229 */  {19, 67, 0, 19, 71, 0, "11"},
/* 230 */  {19, 71, 0, 19, 75, 0, "12"},
/* 231 */  {19, 75, 0, 19, 78, 0, "13"},
/* 232 */  {19, 78, 0, 19, 82, 0, "14"},
/* 233 */  {19, 82, 0, 19, 86, 0, "15"},
/* 234 */  {19, 86, 0, 19, 90, 0, "16"},
/* 235 */  {19, 90, 0, 19, 94, 0, "17"},
/* 236 */  {19, 94, 0, 19, 98, 0, "18"},
/* 237 */  {19, 98, 0, 19, 102, 0, "19"},
/* 238 */  {19, 102, 0, 19, 106, 0, "20"},
/* 239 */  {19, 106, 0, 19, 110, 0, "21"},
/* 240 */  {19, 110, 0, 19, 114, 0, "22"},
/* 241 */  {19, 114, 0, 19, 118, 0, "23"},
/* 242 */  {19, 118, 0, 19, 122, 0, "24"},
/* 243 */  {19, 122, 0, 19, 127, 0, "25"},
/* 244 */  {19, 127, 0, 19, 131, 0, "26"},
/* 245 */  {19, 131, 0, 19, 135, 0, "27"},
/* 246 */  {19, 135, 0, 19, 140, 0, "28"},
/* 247 */  {19, 140, 0, 19, 144, 0, "29"},
/* 248 */  {19, 144, 0, 19, 148, 0, "30"},
/* 249 */  {19, 148, 0, 19, 153, 0, "This is object number"},
/* 250 */  {19, 153, 0, 19, 158, 0, "This is location number"},
/* 251 */  {19, 158, 0, 19, 163, 0, "This door number"},
/* 252 */  {19, 163, 0, 19, 169, 0, "This is person number"},
/* 253 */  {19, 169, 0, 19, 174, 0, "This is day number"},

/* 254 */  {20, 0, 0, 20, 3, 0, "Please follow me"},
/* 255 */  {20, 3, 0, 20, 7, 0, "Would you please follow me"},
/* 256 */  {20, 7, 0, 20, 11, 0, "Would you please FOLLOW me"},
/* 257 */  {20, 11, 0, 20, 14, 0, "Open the door for me"},
/* 258 */  {20, 14, 0, 20, 16, 0, "Open the door"},
/* 259 */  {20, 16, 0, 20, 20, 0, "Would you please open the door"},
/* 260 */  {20, 20, 0, 20, 24, 0, "Do you want more information"},
/* 261 */  {20, 24, 0, 20, 28, 0, "I do not have more information"},
/* 262 */  {20, 28, 0, 20, 31, 0, "This is all I know"},
/* 263 */  {20, 31, 0, 20, 37, 0, "No more information available"},
/* 264 */  {20, 37, 0, 20, 41, 0, "This is the beginning of the tour"},
/* 265 */  {20, 41, 0, 20, 47, 0, "Normally you will only be expected to press the red button"},
/* 266 */  {20, 47, 0, 20, 51, 0, "Press the yellow button for more information"},
/* 267 */  {20, 51, 0, 20, 56, 0, "Press the green button to repeat the information"},
/* 268 */  {20, 56, 0, 20, 60, 0, "Press the blue button to finish the tour"},
/* 269 */  {20, 60, 0, 20, 67, 0, "Press the red button for more information"},
/* 270 */  {20, 67, 0, 20, 73, 0, "Press the red button to move to the next station"},
/* 271 */  {20, 73, 0, 20, 79, 0, "Press the yellow button to move on to the next station"},

/* 272 */  {21, 0, 0, 21, 4, 0, "Where is the trash"},
/* 273 */  {21, 4, 0, 21, 7, 0, "I see some trash"},
/* 274 */  {21, 7, 0, 21, 10, 0, "This is an object"},
/* 275 */  {21, 10, 0, 21, 12, 0, "This is trash"},
/* 276 */  {21, 12, 0, 21, 15, 0, "This is a bottle"},
/* 277 */  {21, 15, 0, 21, 18, 0, "This is a paper-wad"},
/* 278 */  {21, 18, 0, 21, 21, 0, "This is a can"},
/* 279 */  {21, 21, 0, 21, 25, 0, "Picking up the trash"},
/* 280 */  {21, 25, 0, 21, 29, 0, "Raising the arm"},
/* 281 */  {21, 29, 0, 21, 32, 0, "Deploying the arm"},
/* 282 */  {21, 32, 0, 21, 35, 0, "Where is the trash-bin"},
/* 283 */  {21, 35, 0, 21, 39, 0, "I see a trash-bin"},
/* 284 */  {21, 39, 0, 21, 44, 0, "I will move to the trash-bin"},
/* 285 */  {21, 44, 0, 21, 47, 0, "I will move to the trash"},
/* 286 */  {21, 47, 0, 21, 52, 0, "I will dump the trash in the trash-bin"},
/* 287 */  {21, 52, 0, 21, 56, 0, "I will drop the trash"},
/* 288 */  {21, 56, 0, 21, 61, 0, "Cleaning operation successfully terminated (~)"},
/* 289 */  {21, 61, 0, 21, 66, 0, "Cleaning operation successful"},
/* 290 */  {21, 66, 0, 21, 70, 0, "Cleaning operation failed"},
/* 291 */  {21, 70, 0, 21, 73, 0, "I am unable to locate trash"},

/* 292 */  {22, 0, 0, 22, 3, 0, "I am going to"},
/* 293 */  {22, 3, 0, 22, 6, 0, "I am dropping off"},
/* 294 */  {22, 6, 0, 22, 8, 0, "I am picking up"},
/* 295 */  {22, 8, 0, 22, 10, 0, "for"},
/* 296 */  {22, 10, 0, 22, 12, 0, "letter"},
/* 297 */  {22, 12, 0, 22, 14, 0, "Dieter"},
/* 298 */  {22, 14, 0, 22, 16, 0, "Sebastian"},
/* 299 */  {22, 16, 0, 22, 18, 0, "class-room"},
/* 300 */  {22, 18, 0, 22, 21, 0, "library"},
/* 301 */  {22, 21, 0, 22, 24, 0, "Do you want more information"},
/* 302 */  {22, 24, 0, 22, 28, 0, "Do you still want more information"},
/* 303 */  {22, 28, 0, 22, 33, 0, "Press the red button for detailed information"},
/* 304 */  {22, 33, 0, 22, 38, 0, "Press the green button for an overview"},
/* 305 */  {22, 38, 0, 22, 42, 0, "Press no button for no information"},
/* 306 */  {22, 42, 0, 22, 50, 0, "Sebastian Thrun was born ..."},
/* 307 */  {22, 50, 0, 22, 55, 0, "I am a robot manufactured by RWI"},
/* 308 */  {22, 55, 0, 22, 58, 0, "I was born two years ago ..."},

/* 309 */  {23,  0, 0, 24,  0, 0, "Theme of James Bond"},
/* 310 */  {24,  0, 0, 25,  0, 0, "Jazz: Carmel (by Joe Sample)"},
/* 311 */  {25,  0, 0, 26,  0, 0, "Huldigungsmarsch (Grieg)"},
/* 312 */  {26,  0, 0, 27,  0, 0, "Vier Jahreszeiten, Spring, Allegro (Vivaldi)"}
};



#else

/* typedef struct { */
/*   int start_track; */
/*   int start_time; */
/*   int start_frame; */
/*   int end_track; */
/*   int end_time; */
/*   int end_frame; */
/*   char *text; */
/* } CD_msg_type; */

CD_msg_type MSGs[NUM_MSGs] = {
/* 0 */  { 1, 0, 0, 1, 42, 0, "Theme of James Bond" },
/* 1 */  { 2, 7, 0, 2, 10, 0, "Trumpet"},
/* 2 */  { 4, 20, 0, 4, 26, 0, "View to a kill"},
/* 3 */  { 4, 40, 0, 4, 42, 0, "TrumpetTrumpet"},
/* 4 */  { 11, 72, 0, 11, 80, 0, "License to kill"},
/* 5 */  { 13, 10, 0, 13, 20, 0, "He always runs ..."},
/* 6 */  { 19, 0, 0, 19, 45, 0, "Karl May featuring 007"}
};

#endif
