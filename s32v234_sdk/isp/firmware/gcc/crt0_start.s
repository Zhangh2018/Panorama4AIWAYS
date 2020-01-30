/**************************************************************************************************
 * 
 * NXP Confidential Proprietary
 *
 * Copyright 2017 NXP 
 * All Rights Reserved
 *
 *****************************************************************************
 *
 * THIS SOFTWARE IS PROVIDED BY NXP "AS IS" AND ANY EXPRESSED OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL NXP OR ITS CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
 * INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
 * IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
 * THE POSSIBILITY OF SUCH DAMAGE.
 *
 **************************************************************************************************/
 
/*
 * File:	crt0_m0_0_start.s
 * Purpose:	Lowest level routines for CM0.
 *
 * Notes:	
 *
 */

.globl _reset
.globl _exception

.extern _comp_init
.extern main
.extern _done
.extern __CM0_0_STACK_ADDRESS
.extern exceptionHL

.thumb
.section .text
_reset:
	/*** disable interrupts ***/
	movs	r0,	#0x1
	msr	PRIMASK,r0
	
	ldr	r0,	=__CM0_0_STACK_ADDRESS	/* set stack */
	mov	sp,	r0

	/*** clear core regs ***/
	movs	r0,	#0x0
	mov	r1,	r0
	mov	r2,	r0
	mov	r3,	r0
	mov	r4,	r0
	mov	r5,	r0
	mov	r6,	r0
	mov	r7,	r0
	mov	r8,	r0
	mov	r9,	r0
	mov	r10,	r0
	mov	r11,	r0
	mov	r12,	r0

	movs	r0,	#0x0	/* parameter to main */
	BL	_comp_init	/* call compiler init code */
	BL	main		/* call the C code */
	/*** shouldnt return ***/
	B	_done

_exception:
    ldr	r0,	[r13, #0x18]
	BL	exceptionHL
	B	_done
