/******************************************************************************
* | File      	:   GUI_Paint.c
* | Author      :   Waveshare electronics
* | Function    :	Achieve drawing: draw points, lines, boxes, circles and
*                   their size, solid dotted line, solid rectangle hollow
*                   rectangle, solid circle hollow circle.
* | Info        :
*   Achieve display characters: Display a single character, string, number
*   Achieve time display: adaptive size display time minutes and seconds
*----------------
* |	This version:   V3.1
* | Date        :   2020-07-08
* | Info        :
* -----------------------------------------------------------------------------
* V3.1(2020-07-08):
* 1.Change: EPD_SetScale(UBYTE scale)
*		 Add scale 7 for 5.65f e-Parper
* 2.Change: EPD_SetPixel(UWORD Xpoint, UWORD Ypoint, UWORD Color)
*		 Add the branch for scale 7
* 3.Change: EPD_Clear(UWORD Color)
*		 Add the branch for scale 7
*
* -----------------------------------------------------------------------------
* V3.0(2019-04-18):
* 1.Change: 
*    EPD_DrawPoint(..., DOT_STYLE DOT_STYLE)
* => EPD_DrawPoint(..., DOT_STYLE Dot_Style)
*    EPD_DrawLine(..., LINE_STYLE Line_Style, DOT_PIXEL Dot_Pixel)
* => EPD_DrawLine(..., DOT_PIXEL Line_width, LINE_STYLE Line_Style)
*    EPD_DrawRectangle(..., DRAW_FILL Filled, DOT_PIXEL Dot_Pixel)
* => EPD_DrawRectangle(..., DOT_PIXEL Line_width, DRAW_FILL Draw_Fill)
*    EPD_DrawCircle(..., DRAW_FILL Draw_Fill, DOT_PIXEL Dot_Pixel)
* => EPD_DrawCircle(..., DOT_PIXEL Line_width, DRAW_FILL Draw_Filll)
*
* -----------------------------------------------------------------------------
* V2.0(2018-11-15):
* 1.add: EPD_NewImage()
*    Create an image's properties
* 2.add: EPD_SelectImage()
*    Select the picture to be drawn
* 3.add: EPD_SetRotate()
*    Set the direction of the cache    
* 4.add: EPD_RotateImage() 
*    Can flip the picture, Support 0-360 degrees, 
*    but only 90.180.270 rotation is better
* 4.add: EPD_SetMirroring() 
*    Can Mirroring the picture, horizontal, vertical, origin
* 5.add: EPD_DrawString_CN() 
*    Can display Chinese(GB1312)   
*
* ----------------------------------------------------------------------------- 
* V1.0(2018-07-17):
*   Create library
*
* Permission is hereby granted, free of charge, to any person obtaining a copy
* of this software and associated documnetation files (the "Software"), to deal
* in the Software without restriction, including without limitation the rights
* to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
* copies of the Software, and to permit persons to  whom the Software is
* furished to do so, subject to the following conditions:
*
* The above copyright notice and this permission notice shall be included in
* all copies or substantial portions of the Software.
*
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
* IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
* FITNESS OR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
* AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
* LIABILITY WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
* OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
* THE SOFTWARE.
*
******************************************************************************/
#include "EPD_GUI.h"
#include "EPD_Conf.h"
#include <stdint.h>
#include <stdlib.h>
#include <string.h> //memset()
#include <math.h>

PAINT Paint;

/******************************************************************************
function: Create Image
parameter:
    image   :   Pointer to the image cache
    width   :   The width of the picture
    Height  :   The height of the picture
    Color   :   Whether the picture is inverted
******************************************************************************/
void EPD_NewImage(UBYTE *image, UWORD Width, UWORD Height, UWORD Rotate, UWORD Color)
{
    Paint.Image = NULL;
    Paint.Image = image;

    Paint.WidthMemory = Width;
    Paint.HeightMemory = Height;
    Paint.Color = Color;    
	Paint.Scale = 2;
		
    Paint.WidthByte = (Width % 8 == 0)? (Width / 8 ): (Width / 8 + 1);
    Paint.HeightByte = Height;    
//    printf("WidthByte = %d, HeightByte = %d\r\n", Paint.WidthByte, Paint.HeightByte);
//    printf(" EPD_WIDTH / 8 = %d\r\n",  122 / 8);
   
    Paint.Rotate = Rotate;
    Paint.Mirror = MIRROR_NONE;
    
    if(Rotate == ROTATE_0 || Rotate == ROTATE_180) {
        Paint.Width = Width;
        Paint.Height = Height;
    } else {
        Paint.Width = Height;
        Paint.Height = Width;
    }
}

/******************************************************************************
function: Select Image
parameter:
    image : Pointer to the image cache
******************************************************************************/
void EPD_SelectImage(UBYTE *image)
{
    Paint.Image = image;
}

/******************************************************************************
function: Select Image Rotate
parameter:
    Rotate : 0,90,180,270
******************************************************************************/
void EPD_SetRotate(UWORD Rotate)
{
    if(Rotate == ROTATE_0 || Rotate == ROTATE_90 || Rotate == ROTATE_180 || Rotate == ROTATE_270) {
        Paint.Rotate = Rotate;
    } else {
    }
}

void EPD_SetScale(UBYTE scale)
{
    if(scale == 2){
        Paint.Scale = scale;
        Paint.WidthByte = (Paint.WidthMemory % 8 == 0)? (Paint.WidthMemory / 8 ): (Paint.WidthMemory / 8 + 1);
    }else if(scale == 4){
        Paint.Scale = scale;
        Paint.WidthByte = (Paint.WidthMemory % 4 == 0)? (Paint.WidthMemory / 4 ): (Paint.WidthMemory / 4 + 1);
    }else if(scale == 7){//Only applicable with 5in65 e-Paper
				Paint.Scale = scale;
				Paint.WidthByte = (Paint.WidthMemory % 2 == 0)? (Paint.WidthMemory / 2 ): (Paint.WidthMemory / 2 + 1);;
		}else{
    }
}
/******************************************************************************
function:	Select Image mirror
parameter:
    mirror   :Not mirror,Horizontal mirror,Vertical mirror,Origin mirror
******************************************************************************/
void EPD_SetMirroring(UBYTE mirror)
{
    if(mirror == MIRROR_NONE || mirror == MIRROR_HORIZONTAL || 
        mirror == MIRROR_VERTICAL || mirror == MIRROR_ORIGIN) {
        Paint.Mirror = mirror;
    } else {

    }    
}

/******************************************************************************
function: Draw Pixels
parameter:
    Xpoint : At point X
    Ypoint : At point Y
    Color  : Painted colors
******************************************************************************/
void EPD_SetPixel(UWORD Xpoint, UWORD Ypoint, UWORD Color)
{
    if(Xpoint > Paint.Width || Ypoint > Paint.Height){
        return;
    }      
    UWORD X, Y;

    switch(Paint.Rotate) {
    case 0:
        X = Xpoint;
        Y = Ypoint;  
        break;
    case 90:
        X = Paint.WidthMemory - Ypoint - 1;
        Y = Xpoint;
        break;
    case 180:
        X = Paint.WidthMemory - Xpoint - 1;
        Y = Paint.HeightMemory - Ypoint - 1;
        break;
    case 270:
        X = Ypoint;
        Y = Paint.HeightMemory - Xpoint - 1;
        break;
    default:
        return;
    }
    
    switch(Paint.Mirror) {
    case MIRROR_NONE:
        break;
    case MIRROR_HORIZONTAL:
        X = Paint.WidthMemory - X - 1;
        break;
    case MIRROR_VERTICAL:
        Y = Paint.HeightMemory - Y - 1;
        break;
    case MIRROR_ORIGIN:
        X = Paint.WidthMemory - X - 1;
        Y = Paint.HeightMemory - Y - 1;
        break;
    default:
        return;
    }

    if(X > Paint.WidthMemory || Y > Paint.HeightMemory){
        return;
    }
    
    if(Paint.Scale == 2){
        UDOUBLE Addr = X / 8 + Y * Paint.WidthByte;
        UBYTE Rdata = Paint.Image[Addr];
        if(Color == BLACK)
            Paint.Image[Addr] = Rdata & ~(0x80 >> (X % 8));
        else
            Paint.Image[Addr] = Rdata | (0x80 >> (X % 8));
    }else if(Paint.Scale == 4){
        UDOUBLE Addr = X / 4 + Y * Paint.WidthByte;
        Color = Color % 4;//Guaranteed color scale is 4  --- 0~3
        UBYTE Rdata = Paint.Image[Addr];
        
        Rdata = Rdata & (~(0xC0 >> ((X % 4)*2)));
        Paint.Image[Addr] = Rdata | ((Color << 6) >> ((X % 4)*2));
    }else if(Paint.Scale == 7){
		UDOUBLE Addr = X / 2  + Y * Paint.WidthByte;
		UBYTE Rdata = Paint.Image[Addr];
		Rdata = Rdata & (~(0xF0 >> ((X % 2)*4)));//Clear first, then set value
		Paint.Image[Addr] = Rdata | ((Color << 4) >> ((X % 2)*4));
		//printf("Add =  %d ,data = %d\r\n",Addr,Rdata);
		}
}

/******************************************************************************
function: Clear the color of the picture
parameter:
    Color : Painted colors
******************************************************************************/
void EPD_Clear(UWORD Color)
{
	if(Paint.Scale == 2) {
		for (UWORD Y = 0; Y < Paint.HeightByte; Y++) {
			for (UWORD X = 0; X < Paint.WidthByte; X++ ) {//8 pixel =  1 byte
				UDOUBLE Addr = X + Y*Paint.WidthByte;
				Paint.Image[Addr] = Color;
			}
		}		
    }else if(Paint.Scale == 4) {
        for (UWORD Y = 0; Y < Paint.HeightByte; Y++) {
			for (UWORD X = 0; X < Paint.WidthByte; X++ ) {
				UDOUBLE Addr = X + Y*Paint.WidthByte;
				Paint.Image[Addr] = (Color<<6)|(Color<<4)|(Color<<2)|Color;
			}
		}		
	}else if(Paint.Scale == 7) {
		for (UWORD Y = 0; Y < Paint.HeightByte; Y++) {
			for (UWORD X = 0; X < Paint.WidthByte; X++ ) {
				UDOUBLE Addr = X + Y*Paint.WidthByte;
				Paint.Image[Addr] = (Color<<4)|Color;
			}
		}		
	}
}

/******************************************************************************
function: Clear the color of a window
parameter:
    Xstart : x starting point
    Ystart : Y starting point
    Xend   : x end point
    Yend   : y end point
    Color  : Painted colors
******************************************************************************/
void EPD_ClearWindows(UWORD Xstart, UWORD Ystart, UWORD Xend, UWORD Yend, UWORD Color)
{
    UWORD X, Y;
    for (Y = Ystart; Y < Yend; Y++) {
        for (X = Xstart; X < Xend; X++) {//8 pixel =  1 byte
            EPD_SetPixel(X, Y, Color);
        }
    }
}

/******************************************************************************
function: Draw Point(Xpoint, Ypoint) Fill the color
parameter:
    Xpoint		: The Xpoint coordinate of the point
    Ypoint		: The Ypoint coordinate of the point
    Color		: Painted color
    Dot_Pixel	: point size
    Dot_Style	: point Style
******************************************************************************/
void EPD_DrawPoint(UWORD Xpoint, UWORD Ypoint, UWORD Color,
                     DOT_PIXEL Dot_Pixel, DOT_STYLE Dot_Style)
{
    if (Xpoint > Paint.Width || Ypoint > Paint.Height) {
        return;
    }

    int16_t XDir_Num , YDir_Num;
    if (Dot_Style == DOT_FILL_AROUND) {
        for (XDir_Num = 0; XDir_Num < 2 * Dot_Pixel - 1; XDir_Num++) {
            for (YDir_Num = 0; YDir_Num < 2 * Dot_Pixel - 1; YDir_Num++) {
                if(Xpoint + XDir_Num - Dot_Pixel < 0 || Ypoint + YDir_Num - Dot_Pixel < 0)
                    break;
                // printf("x = %d, y = %d\r\n", Xpoint + XDir_Num - Dot_Pixel, Ypoint + YDir_Num - Dot_Pixel);
                EPD_SetPixel(Xpoint + XDir_Num - Dot_Pixel, Ypoint + YDir_Num - Dot_Pixel, Color);
            }
        }
    } else {
        for (XDir_Num = 0; XDir_Num <  Dot_Pixel; XDir_Num++) {
            for (YDir_Num = 0; YDir_Num <  Dot_Pixel; YDir_Num++) {
                EPD_SetPixel(Xpoint + XDir_Num - 1, Ypoint + YDir_Num - 1, Color);
            }
        }
    }
}

/******************************************************************************
function: Draw a line of arbitrary slope
parameter:
    Xstart ：Starting Xpoint point coordinates
    Ystart ：Starting Xpoint point coordinates
    Xend   ：End point Xpoint coordinate
    Yend   ：End point Ypoint coordinate
    Color  ：The color of the line segment
    Line_width : Line width
    Line_Style: Solid and dotted lines
******************************************************************************/
void EPD_DrawLine(UWORD Xstart, UWORD Ystart, UWORD Xend, UWORD Yend,
                    UWORD Color, DOT_PIXEL Line_width, LINE_STYLE Line_Style)
{
    if (Xstart > Paint.Width || Ystart > Paint.Height ||
        Xend > Paint.Width || Yend > Paint.Height) {
        return;
    }

    UWORD Xpoint = Xstart;
    UWORD Ypoint = Ystart;
    int dx = (int)Xend - (int)Xstart >= 0 ? Xend - Xstart : Xstart - Xend;
    int dy = (int)Yend - (int)Ystart <= 0 ? Yend - Ystart : Ystart - Yend;

    // Increment direction, 1 is positive, -1 is counter;
    int XAddway = Xstart < Xend ? 1 : -1;
    int YAddway = Ystart < Yend ? 1 : -1;

    //Cumulative error
    int Esp = dx + dy;
    char Dotted_Len = 0;

    for (;;) {
        Dotted_Len++;
        //Painted dotted line, 2 point is really virtual
        if (Line_Style == LINE_STYLE_DOTTED && Dotted_Len % 3 == 0) {
            //Debug("LINE_DOTTED\r\n");
            EPD_DrawPoint(Xpoint, Ypoint, IMAGE_BACKGROUND, Line_width, DOT_STYLE_DFT);
            Dotted_Len = 0;
        } else {
            EPD_DrawPoint(Xpoint, Ypoint, Color, Line_width, DOT_STYLE_DFT);
        }
        if (2 * Esp >= dy) {
            if (Xpoint == Xend)
                break;
            Esp += dy;
            Xpoint += XAddway;
        }
        if (2 * Esp <= dx) {
            if (Ypoint == Yend)
                break;
            Esp += dx;
            Ypoint += YAddway;
        }
    }
}

/******************************************************************************
function: Draw a rectangle
parameter:
    Xstart ：Rectangular  Starting Xpoint point coordinates
    Ystart ：Rectangular  Starting Xpoint point coordinates
    Xend   ：Rectangular  End point Xpoint coordinate
    Yend   ：Rectangular  End point Ypoint coordinate
    Color  ：The color of the Rectangular segment
    Line_width: Line width
    Draw_Fill : Whether to fill the inside of the rectangle
******************************************************************************/
void EPD_DrawRectangle(UWORD Xstart, UWORD Ystart, UWORD Xend, UWORD Yend,
                         UWORD Color, DOT_PIXEL Line_width, DRAW_FILL Draw_Fill)
{
    if (Xstart > Paint.Width || Ystart > Paint.Height ||
        Xend > Paint.Width || Yend > Paint.Height) {
        return;
    }

    if (Draw_Fill) {
        UWORD Ypoint;
        for(Ypoint = Ystart; Ypoint < Yend; Ypoint++) {
            EPD_DrawLine(Xstart, Ypoint, Xend, Ypoint, Color , Line_width, LINE_STYLE_SOLID);
        }
    } else {
        EPD_DrawLine(Xstart, Ystart, Xend, Ystart, Color, Line_width, LINE_STYLE_SOLID);
        EPD_DrawLine(Xstart, Ystart, Xstart, Yend, Color, Line_width, LINE_STYLE_SOLID);
        EPD_DrawLine(Xend, Yend, Xend, Ystart, Color, Line_width, LINE_STYLE_SOLID);
        EPD_DrawLine(Xend, Yend, Xstart, Yend, Color, Line_width, LINE_STYLE_SOLID);
    }
}

/******************************************************************************
function: Use the 8-point method to draw a circle of the
            specified size at the specified position->
parameter:
    X_Center  ：Center X coordinate
    Y_Center  ：Center Y coordinate
    Radius    ：circle Radius
    Color     ：The color of the ：circle segment
    Line_width: Line width
    Draw_Fill : Whether to fill the inside of the Circle
******************************************************************************/
void EPD_DrawCircle(UWORD X_Center, UWORD Y_Center, UWORD Radius,
                      UWORD Color, DOT_PIXEL Line_width, DRAW_FILL Draw_Fill)
{
    if (X_Center > Paint.Width || Y_Center >= Paint.Height) {
        return;
    }

    //Draw a circle from(0, R) as a starting point
    int16_t XCurrent, YCurrent;
    XCurrent = 0;
    YCurrent = Radius;

    //Cumulative error,judge the next point of the logo
    int16_t Esp = 3 - (Radius << 1 );

    int16_t sCountY;
    if (Draw_Fill == DRAW_FILL_FULL) {
        while (XCurrent <= YCurrent ) { //Realistic circles
            for (sCountY = XCurrent; sCountY <= YCurrent; sCountY ++ ) {
                EPD_DrawPoint(X_Center + XCurrent, Y_Center + sCountY, Color, DOT_PIXEL_DFT, DOT_STYLE_DFT);//1
                EPD_DrawPoint(X_Center - XCurrent, Y_Center + sCountY, Color, DOT_PIXEL_DFT, DOT_STYLE_DFT);//2
                EPD_DrawPoint(X_Center - sCountY, Y_Center + XCurrent, Color, DOT_PIXEL_DFT, DOT_STYLE_DFT);//3
                EPD_DrawPoint(X_Center - sCountY, Y_Center - XCurrent, Color, DOT_PIXEL_DFT, DOT_STYLE_DFT);//4
                EPD_DrawPoint(X_Center - XCurrent, Y_Center - sCountY, Color, DOT_PIXEL_DFT, DOT_STYLE_DFT);//5
                EPD_DrawPoint(X_Center + XCurrent, Y_Center - sCountY, Color, DOT_PIXEL_DFT, DOT_STYLE_DFT);//6
                EPD_DrawPoint(X_Center + sCountY, Y_Center - XCurrent, Color, DOT_PIXEL_DFT, DOT_STYLE_DFT);//7
                EPD_DrawPoint(X_Center + sCountY, Y_Center + XCurrent, Color, DOT_PIXEL_DFT, DOT_STYLE_DFT);
            }
            if (Esp < 0 )
                Esp += 4 * XCurrent + 6;
            else {
                Esp += 10 + 4 * (XCurrent - YCurrent );
                YCurrent --;
            }
            XCurrent ++;
        }
    } else { //Draw a hollow circle
        while (XCurrent <= YCurrent ) {
            EPD_DrawPoint(X_Center + XCurrent, Y_Center + YCurrent, Color, Line_width, DOT_STYLE_DFT);//1
            EPD_DrawPoint(X_Center - XCurrent, Y_Center + YCurrent, Color, Line_width, DOT_STYLE_DFT);//2
            EPD_DrawPoint(X_Center - YCurrent, Y_Center + XCurrent, Color, Line_width, DOT_STYLE_DFT);//3
            EPD_DrawPoint(X_Center - YCurrent, Y_Center - XCurrent, Color, Line_width, DOT_STYLE_DFT);//4
            EPD_DrawPoint(X_Center - XCurrent, Y_Center - YCurrent, Color, Line_width, DOT_STYLE_DFT);//5
            EPD_DrawPoint(X_Center + XCurrent, Y_Center - YCurrent, Color, Line_width, DOT_STYLE_DFT);//6
            EPD_DrawPoint(X_Center + YCurrent, Y_Center - XCurrent, Color, Line_width, DOT_STYLE_DFT);//7
            EPD_DrawPoint(X_Center + YCurrent, Y_Center + XCurrent, Color, Line_width, DOT_STYLE_DFT);//0

            if (Esp < 0 )
                Esp += 4 * XCurrent + 6;
            else {
                Esp += 10 + 4 * (XCurrent - YCurrent );
                YCurrent --;
            }
            XCurrent ++;
        }
    }
}

/******************************************************************************
function: Show English characters
parameter:
    Xpoint           ：X coordinate
    Ypoint           ：Y coordinate
    Acsii_Char       ：To display the English characters
    Font             ：A structure pointer that displays a character size
    Color_Foreground : Select the foreground color
    Color_Background : Select the background color
******************************************************************************/
void EPD_DrawChar(UWORD Xpoint, UWORD Ypoint, const char Acsii_Char,
                    sFONT* Font, UWORD Color_Foreground, UWORD Color_Background)
{
    UWORD Page, Column;

    if (Xpoint > Paint.Width || Ypoint > Paint.Height) {
        return;
    }

    uint32_t Char_Offset = (Acsii_Char - ' ') * Font->Height * (Font->Width / 8 + (Font->Width % 8 ? 1 : 0));
    const unsigned char *ptr = &Font->table[Char_Offset];

    for (Page = 0; Page < Font->Height; Page ++ ) {
        for (Column = 0; Column < Font->Width; Column ++ ) {

            //To determine whether the font background color and screen background color is consistent
            if (FONT_BACKGROUND == Color_Background) { //this process is to speed up the scan
                if (*ptr & (0x80 >> (Column % 8)))
                    EPD_SetPixel(Xpoint + Column, Ypoint + Page, Color_Foreground);
                    // EPD_DrawPoint(Xpoint + Column, Ypoint + Page, Color_Foreground, DOT_PIXEL_DFT, DOT_STYLE_DFT);
            } else {
                if (*ptr & (0x80 >> (Column % 8))) {
                    EPD_SetPixel(Xpoint + Column, Ypoint + Page, Color_Foreground);
                    // EPD_DrawPoint(Xpoint + Column, Ypoint + Page, Color_Foreground, DOT_PIXEL_DFT, DOT_STYLE_DFT);
                } else {
                    EPD_SetPixel(Xpoint + Column, Ypoint + Page, Color_Background);
                    // EPD_DrawPoint(Xpoint + Column, Ypoint + Page, Color_Background, DOT_PIXEL_DFT, DOT_STYLE_DFT);
                }
            }
            //One pixel is 8 bits
            if (Column % 8 == 7)
                ptr++;
        }// Write a line
        if (Font->Width % 8 != 0)
            ptr++;
    }// Write all
}

/******************************************************************************
function:	Display the string
parameter:
    Xstart           ：X coordinate
    Ystart           ：Y coordinate
    pString          ：The first address of the English string to be displayed
    Font             ：A structure pointer that displays a character size
    Color_Foreground : Select the foreground color
    Color_Background : Select the background color
******************************************************************************/
void EPD_DrawString_EN(UWORD Xstart, UWORD Ystart, const char * pString,
                         sFONT* Font, UWORD Color_Foreground, UWORD Color_Background)
{
    UWORD Xpoint = Xstart;
    UWORD Ypoint = Ystart;

    if (Xstart > Paint.Width || Ystart > Paint.Height) {
        return;
    }

    while (* pString != '\0') {
        //if X direction filled , reposition to(Xstart,Ypoint),Ypoint is Y direction plus the Height of the character
        if ((Xpoint + Font->Width ) > Paint.Width ) {
            Xpoint = Xstart;
            Ypoint += Font->Height;
        }

        // If the Y direction is full, reposition to(Xstart, Ystart)
        if ((Ypoint  + Font->Height ) > Paint.Height ) {
            Xpoint = Xstart;
            Ypoint = Ystart;
        }
        EPD_DrawChar(Xpoint, Ypoint, * pString, Font, Color_Background, Color_Foreground);

        //The next character of the address
        pString ++;

        //The next word of the abscissa increases the font of the broadband
        Xpoint += Font->Width;
    }
}


/******************************************************************************
function: Display the string
parameter:
    Xstart  ：X coordinate
    Ystart  ：Y coordinate
    pString ：The first address of the Chinese string and English
              string to be displayed
    Font    ：A structure pointer that displays a character size
    Color_Foreground : Select the foreground color
    Color_Background : Select the background color
******************************************************************************/
void EPD_DrawString_CN(UWORD Xstart, UWORD Ystart, const char * pString, cFONT* font,
                        UWORD Color_Foreground, UWORD Color_Background)
{
    const char* p_text = pString;
    int x = Xstart, y = Ystart;
    int i, j,Num;

    /* Send the string character by character on EPD */
    while (*p_text != 0) {
        if(*p_text <= 0x7F) {  //ASCII < 126
            for(Num = 0; Num < font->size; Num++) {
                if(*p_text== font->table[Num].index[0]) {
                    const char* ptr = &font->table[Num].matrix[0];

                    for (j = 0; j < font->Height; j++) {
                        for (i = 0; i < font->Width; i++) {
                            if (FONT_BACKGROUND == Color_Background) { //this process is to speed up the scan
                                if (*ptr & (0x80 >> (i % 8))) {
                                    EPD_SetPixel(x + i, y + j, Color_Foreground);
                                    // EPD_DrawPoint(x + i, y + j, Color_Foreground, DOT_PIXEL_DFT, DOT_STYLE_DFT);
                                }
                            } else {
                                if (*ptr & (0x80 >> (i % 8))) {
                                    EPD_SetPixel(x + i, y + j, Color_Foreground);
                                    // EPD_DrawPoint(x + i, y + j, Color_Foreground, DOT_PIXEL_DFT, DOT_STYLE_DFT);
                                } else {
                                    EPD_SetPixel(x + i, y + j, Color_Background);
                                    // EPD_DrawPoint(x + i, y + j, Color_Background, DOT_PIXEL_DFT, DOT_STYLE_DFT);
                                }
                            }
                            if (i % 8 == 7) {
                                ptr++;
                            }
                        }
                        if (font->Width % 8 != 0) {
                            ptr++;
                        }
                    }
                    break;
                }
            }
            /* Point on the next character */
            p_text += 1;
            /* Decrement the column position by 16 */
            x += font->ASCII_Width;
        } else {        //Chinese
            for(Num = 0; Num < font->size; Num++) {
                if((*p_text== font->table[Num].index[0]) && (*(p_text+1) == font->table[Num].index[1])) {
                    const char* ptr = &font->table[Num].matrix[0];

                    for (j = 0; j < font->Height; j++) {
                        for (i = 0; i < font->Width; i++) {
                            if (FONT_BACKGROUND == Color_Background) { //this process is to speed up the scan
                                if (*ptr & (0x80 >> (i % 8))) {
                                    EPD_SetPixel(x + i, y + j, Color_Foreground);
                                    // EPD_DrawPoint(x + i, y + j, Color_Foreground, DOT_PIXEL_DFT, DOT_STYLE_DFT);
                                }
                            } else {
                                if (*ptr & (0x80 >> (i % 8))) {
                                    EPD_SetPixel(x + i, y + j, Color_Foreground);
                                    // EPD_DrawPoint(x + i, y + j, Color_Foreground, DOT_PIXEL_DFT, DOT_STYLE_DFT);
                                } else {
                                    EPD_SetPixel(x + i, y + j, Color_Background);
                                    // EPD_DrawPoint(x + i, y + j, Color_Background, DOT_PIXEL_DFT, DOT_STYLE_DFT);
                                }
                            }
                            if (i % 8 == 7) {
                                ptr++;
                            }
                        }
                        if (font->Width % 8 != 0) {
                            ptr++;
                        }
                    }
                    break;
                }
            }
            /* Point on the next character */
            p_text += 2;
            /* Decrement the column position by 16 */
            x += font->Width;
        }
    }
}

/******************************************************************************
function:	Display nummber
parameter:
    Xstart           ：X coordinate
    Ystart           : Y coordinate
    Nummber          : The number displayed
    Font             ：A structure pointer that displays a character size
    Color_Foreground : Select the foreground color
    Color_Background : Select the background color
******************************************************************************/
#define  ARRAY_LEN 255
void EPD_DrawNum(UWORD Xpoint, UWORD Ypoint, int32_t Nummber,
                   sFONT* Font, UWORD Color_Foreground, UWORD Color_Background)
{

    int16_t Num_Bit = 0, Str_Bit = 0;
    uint8_t Str_Array[ARRAY_LEN] = {0}, Num_Array[ARRAY_LEN] = {0};
    uint8_t *pStr = Str_Array;

    if (Xpoint > Paint.Width || Ypoint > Paint.Height) {
        return;
    }

    //Converts a number to a string
    do {
        Num_Array[Num_Bit] = Nummber % 10 + '0';
        Num_Bit++;
        Nummber /= 10;
    } while(Nummber);
    

    //The string is inverted
    while (Num_Bit > 0) {
        Str_Array[Str_Bit] = Num_Array[Num_Bit - 1];
        Str_Bit ++;
        Num_Bit --;
    }

    //show
    EPD_DrawString_EN(Xpoint, Ypoint, (const char*)pStr, Font, Color_Background, Color_Foreground);
}

/******************************************************************************
function:	Display nummber (Able to display decimals)
parameter:
    Xstart           ：X coordinate
    Ystart           : Y coordinate
    Nummber          : The number displayed
    Font             ：A structure pointer that displays a character size
    Digit            : Fractional width
    Color_Foreground : Select the foreground color
    Color_Background : Select the background color
******************************************************************************/
void EPD_DrawNumDecimals(UWORD Xpoint, UWORD Ypoint, double Nummber,
                    sFONT* Font, UWORD Digit, UWORD Color_Foreground, UWORD Color_Background)
{
    int16_t Num_Bit = 0, Str_Bit = 0;
    uint8_t Str_Array[ARRAY_LEN] = {0}, Num_Array[ARRAY_LEN] = {0};
    uint8_t *pStr = Str_Array;
	int temp = Nummber;
	float decimals;
	uint8_t i;
    if (Xpoint > Paint.Width || Ypoint > Paint.Height) {
        return;
    }

	if(Digit > 0) {		
		decimals = Nummber - temp;
		for(i=Digit; i > 0; i--) {
			decimals*=10;
		}
		temp = decimals;
		//Converts a number to a string
		for(i=Digit; i>0; i--) {
			Num_Array[Num_Bit] = temp % 10 + '0';
			Num_Bit++;
			temp /= 10;						
		}	
		Num_Array[Num_Bit] = '.';
		Num_Bit++;
	}

	temp = Nummber;
    //Converts a number to a string
    do {
        Num_Array[Num_Bit] = temp % 10 + '0';
        Num_Bit++;
        temp /= 10;
    } while(temp);

    //The string is inverted
    while (Num_Bit > 0) {
        Str_Array[Str_Bit] = Num_Array[Num_Bit - 1];
        Str_Bit ++;
        Num_Bit --;
    }

    //show
    EPD_DrawString_EN(Xpoint, Ypoint, (const char*)pStr, Font, Color_Background, Color_Foreground);
}

/******************************************************************************
function:	Display time
parameter:
    Xstart           ：X coordinate
    Ystart           : Y coordinate
    pTime            : Time-related structures
    Font             ：A structure pointer that displays a character size
    Color_Foreground : Select the foreground color
    Color_Background : Select the background color
******************************************************************************/
void EPD_DrawDate(UWORD Xstart, UWORD Ystart, EPD_TIME *pTime, sFONT* Font,
                    UWORD Color_Foreground, UWORD Color_Background)
{
    uint8_t value[10] = {'0', '1', '2', '3', '4', '5', '6', '7', '8', '9'};

    UWORD Dx = Font->Width;

    //Write data into the cache
    EPD_DrawChar(Xstart                           , Ystart, value[pTime->Day / 10], Font, Color_Background, Color_Foreground);
    EPD_DrawChar(Xstart + Dx                      , Ystart, value[pTime->Day % 10], Font, Color_Background, Color_Foreground);
    EPD_DrawChar(Xstart + Dx  + Dx / 4 + Dx / 2   , Ystart, '.'                    , Font, Color_Background, Color_Foreground);
    EPD_DrawChar(Xstart + Dx * 2 + Dx / 2         , Ystart, value[pTime->Month / 10] , Font, Color_Background, Color_Foreground);
    EPD_DrawChar(Xstart + Dx * 3 + Dx / 2         , Ystart, value[pTime->Month % 10] , Font, Color_Background, Color_Foreground);
    EPD_DrawChar(Xstart + Dx * 4 + Dx / 2 - Dx / 4, Ystart, '.'                    , Font, Color_Background, Color_Foreground);
    EPD_DrawChar(Xstart + Dx * 5                  , Ystart, value[pTime->Year / 10] , Font, Color_Background, Color_Foreground);
    EPD_DrawChar(Xstart + Dx * 6                  , Ystart, value[pTime->Year % 10] , Font, Color_Background, Color_Foreground);
}

void EPD_DrawTime(UWORD Xstart, UWORD Ystart, EPD_TIME *pTime, sFONT* Font,
    UWORD Color_Foreground, UWORD Color_Background)
{
    uint8_t value[10] = {'0', '1', '2', '3', '4', '5', '6', '7', '8', '9'};

    UWORD Dx = Font->Width;

    //Write data into the cache
    EPD_DrawChar(Xstart                           , Ystart, value[pTime->Hour / 10], Font, Color_Background, Color_Foreground);
    EPD_DrawChar(Xstart + Dx                      , Ystart, value[pTime->Hour % 10], Font, Color_Background, Color_Foreground);
    EPD_DrawChar(Xstart + Dx  + Dx / 4 + Dx / 2   , Ystart, ':'                    , Font, Color_Background, Color_Foreground);
    EPD_DrawChar(Xstart + Dx * 2 + Dx / 2         , Ystart, value[pTime->Min / 10] , Font, Color_Background, Color_Foreground);
    EPD_DrawChar(Xstart + Dx * 3 + Dx / 2         , Ystart, value[pTime->Min % 10] , Font, Color_Background, Color_Foreground);
}

/******************************************************************************
function:	Display monochrome bitmap
parameter:
    image_buffer ：A picture data converted to a bitmap
info:
    Use a computer to convert the image into a corresponding array,
    and then embed the array directly into Imagedata.cpp as a .c file.
******************************************************************************/
void EPD_DrawBitMap(const unsigned char* image_buffer)
{
    UWORD x, y;
    UDOUBLE Addr = 0;

    for (y = 0; y < Paint.HeightByte; y++) {
        for (x = 0; x < Paint.WidthByte; x++) {//8 pixel =  1 byte
            Addr = x + y * Paint.WidthByte;
            Paint.Image[Addr] = (unsigned char)image_buffer[Addr];
        }
    }
}

/******************************************************************************
function:	paste monochrome bitmap to a frame buff
parameter:
    image_buffer ：A picture data converted to a bitmap
    xStart: The starting x coordinate
    yStart: The starting y coordinate
    imageWidth: Original image width
    imageHeight: Original image height
    flipColor: Whether the color is reversed
info:
    Use this function to paste image data into a buffer
******************************************************************************/
void EPD_DrawBitMap_Paste(const unsigned char* image_buffer, UWORD xStart, UWORD yStart, UWORD imageWidth, UWORD imageHeight, UBYTE flipColor)
{
    UBYTE color, srcImage;
    UWORD x, y;
    UWORD width = (imageWidth%8==0 ? imageWidth/8 : imageWidth/8+1);
    
    for (y = 0; y < imageHeight; y++) {
        for (x = 0; x < imageWidth; x++) {
            srcImage = image_buffer[y*width + x/8];
            if(flipColor)
                color = (((srcImage<<(x%8) & 0x80) == 0) ? 1 : 0);
            else
                color = (((srcImage<<(x%8) & 0x80) == 0) ? 0 : 1);
            EPD_SetPixel(x+xStart, y+yStart, color);
        }
    }
}

///******************************************************************************
//function:	SDisplay half of monochrome bitmap
//parameter:
//	Region : 1 Upper half
//					 2 Lower half
//info:
//******************************************************************************/
//void EPD_DrawBitMap_Half(const unsigned char* image_buffer, UBYTE Region)
//{
//    UWORD x, y;
//    UDOUBLE Addr = 0;
//		
//		if(Region == 1){
//			for (y = 0; y < Paint.HeightByte; y++) {
//					for (x = 0; x < Paint.WidthByte; x++) {//8 pixel =  1 byte
//							Addr = x + y * Paint.WidthByte;
//							Paint.Image[Addr] = (unsigned char)image_buffer[Addr];
//					}
//			}
//		}else{
//			for (y = 0; y < Paint.HeightByte; y++) {
//					for (x = 0; x < Paint.WidthByte; x++) {//8 pixel =  1 byte
//							Addr = x + y * Paint.WidthByte ;
//							Paint.Image[Addr] = 
//							(unsigned char)image_buffer[Addr+ (Paint.HeightByte)*Paint.WidthByte];
//					}
//			}
//		}
//}

///******************************************************************************
//function:	SDisplay half of monochrome bitmap
//parameter:
//	Region : 1 Upper half
//					 2 Lower half
//info:
//******************************************************************************/
//void EPD_DrawBitMap_OneQuarter(const unsigned char* image_buffer, UBYTE Region)
//{
//    UWORD x, y;
//    UDOUBLE Addr = 0;
//		
//		if(Region == 1){
//			for (y = 0; y < Paint.HeightByte; y++) {
//					for (x = 0; x < Paint.WidthByte; x++) {//8 pixel =  1 byte
//							Addr = x + y * Paint.WidthByte;
//							Paint.Image[Addr] = (unsigned char)image_buffer[Addr];
//					}
//			}
//		}else if(Region == 2){
//			for (y = 0; y < Paint.HeightByte; y++) {
//					for (x = 0; x < Paint.WidthByte; x++) {//8 pixel =  1 byte
//							Addr = x + y * Paint.WidthByte ;
//							Paint.Image[Addr] = 
//							(unsigned char)image_buffer[Addr+ (Paint.HeightByte)*Paint.WidthByte];
//					}
//			}
//		}else if(Region == 3){
//			for (y = 0; y < Paint.HeightByte; y++) {
//					for (x = 0; x < Paint.WidthByte; x++) {//8 pixel =  1 byte
//							Addr = x + y * Paint.WidthByte ;
//							Paint.Image[Addr] = 
//							(unsigned char)image_buffer[Addr+ (Paint.HeightByte)*Paint.WidthByte*2];
//					}
//			}
//		}else if(Region == 4){
//			for (y = 0; y < Paint.HeightByte; y++) {
//					for (x = 0; x < Paint.WidthByte; x++) {//8 pixel =  1 byte
//							Addr = x + y * Paint.WidthByte ;
//							Paint.Image[Addr] = 
//							(unsigned char)image_buffer[Addr+ (Paint.HeightByte)*Paint.WidthByte*3];
//					}
//			}
//		}
//}

void EPD_DrawBitMap_Block(const unsigned char* image_buffer, UBYTE Region)
{
    UWORD x, y;
    UDOUBLE Addr = 0;
		for (y = 0; y < Paint.HeightByte; y++) {
				for (x = 0; x < Paint.WidthByte; x++) {//8 pixel =  1 byte
						Addr = x + y * Paint.WidthByte ;
						Paint.Image[Addr] = \
						(unsigned char)image_buffer[Addr+ (Paint.HeightByte)*Paint.WidthByte*(Region - 1)];
				}
		}
}

void EPD_DrawBMP(uint8_t xs, uint8_t ys, uint8_t xl, uint8_t yl, uint8_t bmp[xl][yl]){
    for (uint8_t y = 0; y < yl; y++){
      for (uint8_t x = 0; x < xl; x++){
        if(bmp[x][y] == 0){
          EPD_SetPixel(x+xs, y+ys, BLACK);
        }
      }
    }
  }

