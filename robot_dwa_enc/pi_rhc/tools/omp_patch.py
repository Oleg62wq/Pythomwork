#!/usr/bin/env python3
"""Parallelise the candidate loop of RHC raspberry_pi/src/dwa.c with OpenMP.

    python3 omp_patch.py path/to/dwa.c

Only the triple loop over (vx, vy, w) candidates is replaced; everything else
(vycap, weights, checks) stays as it is in the file, so your own edits survive.
Same result as the sequential loop: on equal cost the smaller candidate number
(= the one met first by i, j, k) wins. Without -fopenmp it is still sequential.
Run twice = no change (marker OMP_PATCH).
"""
import sys

OLD = """ double best=1e12;int found=0;
 for(int i=0;i<nvx;i++)for(int j=0;j<nvy;j++)for(int k=0;k<nw;k++){
  double c[3]={i==nvx-1?hi[0]:lo[0]+(hi[0]-lo[0])*i/(nvx-1),
   j==nvy-1?hi[1]:lo[1]+(hi[1]-lo[1])*j/(nvy-1),
   k==nw-1?hi[2]:lo[2]+(hi[2]-lo[2])*k/(nw-1)};
  if(!wheels_ok(in,c,reference))continue;
  double J;if(!rollout(s,in,c,qphi,qy,qd,qvis,1,&J))continue;
  J+=1.20*(sq(c[0]-vxref)+1.5*sq(c[1]-vyref))+.25*sq(c[2]-wref);
  J+=sq(c[0]-vxprev)+.5*sq(c[1]-vyprev)+.4*sq(c[2]-wprev)+2*sq(c[1]);
  if(ph*c[2]<0)J+=8*sq(ph*c[2]);
  out->candidates++;
  if(J<best){best=J;memcpy(out->command,c,sizeof(c));found=1;}
 }
"""

NEW = """ /* OMP_PATCH: candidates in parallel, same result as the sequential loop */
 double best=1e12;int found=0,best_idx=-1,ncand=0;
 const int ntot=nvx*nvy*nw;
#ifdef _OPENMP
 #pragma omp parallel
#endif
 {
  double my_best=1e12;int my_idx=-1,my_cand=0;
#ifdef _OPENMP
  #pragma omp for schedule(dynamic,4) nowait
#endif
  for(int idx=0;idx<ntot;idx++){
   const int k=idx%nw,j=(idx/nw)%nvy,i=idx/(nw*nvy);
   double c[3]={i==nvx-1?hi[0]:lo[0]+(hi[0]-lo[0])*i/(nvx-1),
    j==nvy-1?hi[1]:lo[1]+(hi[1]-lo[1])*j/(nvy-1),
    k==nw-1?hi[2]:lo[2]+(hi[2]-lo[2])*k/(nw-1)};
   if(!wheels_ok(in,c,reference))continue;
   double J;if(!rollout(s,in,c,qphi,qy,qd,qvis,1,&J))continue;
   J+=1.20*(sq(c[0]-vxref)+1.5*sq(c[1]-vyref))+.25*sq(c[2]-wref);
   J+=sq(c[0]-vxprev)+.5*sq(c[1]-vyprev)+.4*sq(c[2]-wprev)+2*sq(c[1]);
   if(ph*c[2]<0)J+=8*sq(ph*c[2]);
   my_cand++;
   if(J<my_best){my_best=J;my_idx=idx;}
  }
#ifdef _OPENMP
  #pragma omp critical(rhc_dwa_best)
#endif
  {
   ncand+=my_cand;
   if(my_idx>=0&&(my_best<best||(my_best==best&&(best_idx<0||my_idx<best_idx)))){best=my_best;best_idx=my_idx;}
  }
 }
 out->candidates+=ncand;
 if(best_idx>=0){
  const int k=best_idx%nw,j=(best_idx/nw)%nvy,i=best_idx/(nw*nvy);
  out->command[0]=i==nvx-1?hi[0]:lo[0]+(hi[0]-lo[0])*i/(nvx-1);
  out->command[1]=j==nvy-1?hi[1]:lo[1]+(hi[1]-lo[1])*j/(nvy-1);
  out->command[2]=k==nw-1?hi[2]:lo[2]+(hi[2]-lo[2])*k/(nw-1);
  found=1;
 }
"""

p = sys.argv[1]
s = open(p).read()
if "OMP_PATCH" in s:
    print("dwa.c: OpenMP already in")
elif OLD in s:
    open(p, "w").write(s.replace(OLD, NEW, 1))
    print("dwa.c: candidate loop now parallel (OpenMP)")
else:
    sys.exit("dwa.c: candidate loop not recognised (changed by hand?) - left sequential")
