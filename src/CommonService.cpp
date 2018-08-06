/*
  Copyright (c) 2017, COMAU S.p.A.
  All rights reserved.
  
  Redistribution and use in source and binary forms, with or without
  modification, are permitted provided that the following conditions are met:
  
  1. Redistributions of source code must retain the above copyright notice, this
     list of conditions and the following disclaimer.
  2. Redistributions in binary form must reproduce the above copyright notice,
     this list of conditions and the following disclaimer in the documentation
     and/or other materials provided with the distribution.
  
  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
  ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
  WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
  ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
  (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
  ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  
  The views and conclusions contained in the software and documentation are those
  of the authors and should not be interpreted as representing official policies,
  either expressed or implied, of the FreeBSD Project.
*/
/*
 * CommonService.cpp
 *
 *  Created on: Jul 12, 2017
 *      Author: comau
 */

#include "CommonService.h"

// count the number of bits set in mask
unsigned int CommonService::countBitMask (unsigned long asm_mask) {
  unsigned int sj_num_bits; 
  unsigned int sj_last_joint;
  unsigned int sj_i;

  asm_mask &= SSM_JOINTS_MASK;
  if (asm_mask == 0)
    return (0);

  for (sj_i = 0, sj_num_bits = 0, sj_last_joint = 0; asm_mask != 0; asm_mask >>= 1, sj_i++)
  {
    if (asm_mask & 1) {
      sj_num_bits += 1;    // accumulates the total bits set in mask
      sj_last_joint = sj_i+1; // it can be the last
    }
  }

  return sj_num_bits;
}

unsigned int CommonService::getLastJointAxis (unsigned long asm_mask) {
  unsigned int sj_num_bits; 
  unsigned int sj_last_joint;
  unsigned int sj_i;
  
  asm_mask &= SSM_JOINTS_MASK;
  if (asm_mask == 0)
    return (0);
  
  for (sj_i = 0, sj_num_bits = 0, sj_last_joint = 0; asm_mask != 0; asm_mask >>= 1, sj_i++)
  {
    if (asm_mask & 1) {
      sj_num_bits += 1;    // accumulates the total bits set in mask
      sj_last_joint = sj_i+1; // it can be the last
    }
  }

  if (sj_last_joint > SSM_NUM_MAX_JOINTS)
    sj_last_joint = SSM_NUM_MAX_JOINTS;
  return sj_last_joint;
}
