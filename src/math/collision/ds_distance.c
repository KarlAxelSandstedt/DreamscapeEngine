
/*
==========================================================================
    Copyright (C) 2026 Axel Sandstedt 

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <https://www.gnu.org/licenses/>.
==========================================================================
*/

#include "collision.h"
#include "ds_barycentric.h"

/*
GJK Implementation: 

Read: TODO 
     (O) Erin-Catto gjk 2010
     (O) Ericcson 3.4
     (O) Ericcson 5.1.5 (Solve3)
     (O) Ericcson 5.1.6 (Solve4)
     () Theory and termination: Gino, collision detection in interactive 3d env. 4.3.1-4.3.8
     (O) Erin-Catto triangles (FP-precision)
     (O) Erin Catto continous collision 2013 (where the cache leads)

Notes:
*/

/*
Erin-Catto GJK 2010
===================

Topic: Baricentric coordinates

TODO: WHen implementing these algorithms, each function should have a small explanation
    of the math, and/or references to external source (such as this presentation).

TODO: When implementing these algorithms, it will serve as a perfect test for code zones
    the Agent should repeatedly search for bugs, simplifications, performance improvements
    , and numerical accuracy/stability.

Line: fractional length. Here Barycentric == Voroni, and we have the SegmentPointBc. 

TODO
Triangle: 
    (A, u) => u = Area(QBC) / Area(ABC)
    (B, v) => v = Area(QCA) / Area(ABC)
    (C, w) => 1 - u - v

    The algorithm here seems to justify an API why can be re-used all over the engine.
    I believe I've already coded this stuff (possibly multiple times), in which we 
    should refactor this into a single, shared API.
    TriangleBC***
    TriangleVoronoi***

TODO: For the triangle case, it is somewhat intuitive to understand the below formulas,
    but why exactly? Sure, the sub-triangle areas add up to the triangle area, but why is
    that weighted exactly as the barycentric coordinates?
 */


/*
TODO: GJK: do you cache the final simplex in previous calls to use as arbitrary simplex at
    the start of next GJK call?
*/
