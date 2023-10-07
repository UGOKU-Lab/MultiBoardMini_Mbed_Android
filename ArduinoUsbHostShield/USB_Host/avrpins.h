/* Copyright (C) 2011 Circuits At Home, LTD. All rights reserved.

This program is free software; you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation; either version 2 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program; if not, write to the Free Software
Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA

Contact information
-------------------

Circuits At Home, LTD
Web      :  http://www.circuitsathome.com
e-mail   :  support@circuitsathome.com
 */

/* derived from Konstantin Chizhov's AVR port templates */

/*
warning
#define _usb_h_
#define MBED_H
*/

#if !defined(_usb_h_) || defined(_avrpins_h_)
#error "Never include avrpins.h directly; include Usb.h instead"
#else
#define _avrpins_h_

#if defined(MBED_H)
// for mbed 
// pointers are 32 bits on ARM
#define pgm_read_pointer(p) pgm_read_dword(p)

#endif

#endif //_avrpins_h_
