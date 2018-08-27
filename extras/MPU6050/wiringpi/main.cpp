/* 
   main.pp: Generic main() for caling Arduino-style setup(), loop()

   This file is part of MPU6050.

   MPU6050 is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   MPU6050 is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.
   You should have received a copy of the GNU General Public License
   along with MPU6050.  If not, see <http://www.gnu.org/licenses/>.
 */

extern void setup(), loop();

int main(int argc, char ** argv)
{
    setup();

    while (true) {
        loop();
    }
}
