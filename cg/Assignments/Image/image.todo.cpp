#include <algorithm>
#include "image.h"
#include <stdlib.h>
#include <random>
#include <math.h>
#include <Util/exceptions.h>
#include <bits/stdc++.h>


using namespace Util;
using namespace Image;
using namespace std;

/////////////
// Image32 //
/////////////
Image32 Image32::addRandomNoise( double noise ) const {

	Image32 output;

	output.setSize(this->_width, this->_height);


	double lower = -noise;
	double upper = noise;

	uniform_real_distribution<double> unif(lower, upper);

	default_random_engine re;


	for (int i = 0; i < this->_width; i++) {
		for (int j = 0; j < this->_height; j++) {
			Pixel32 p = (*this)(i,j);

			double rand_noise = unif(re);

			Pixel32& op = output(i, j);

			u_int temp_r = p.r;
			u_int temp_g = p.g;
			u_int temp_b = p.b;

			op.r = (unsigned char) clamp(int(temp_r + (rand_noise * 255)), 0, 255);
			op.g = (unsigned char) clamp(int(temp_g + (rand_noise * 255)), 0, 255);
			op.b = (unsigned char) clamp(int(temp_b + (rand_noise * 255)), 0, 255);
			op.a = p.a;
		}
	}

	return output;
}

Image32 Image32::brighten( double brightness ) const {
		
	Image32 out;

	out.setSize(this->_width, this->_height);


	for (int i = 0; i < this->_width; i++) {
		for (int j = 0; j < this->_height; j++) {
			Pixel32 p = (*this)(i, j); 

			Pixel32& out_pixel = out(i, j);

			out_pixel.r = fmin(p.r * brightness, 255);
			out_pixel.g = fmin(p.g * brightness, 255);
			out_pixel.b = fmin(p.b * brightness, 255);
			out_pixel.a = p.a;

		}
	}

	return out;
}

Image32 Image32::luminance( void ) const {
	Image32 out;

	out.setSize(this->_width, this->_height);

	for (int i = 0; i < this->_width; i++) {
		for (int j = 0; j < this->_height; j++) {
			Pixel32 p = (*this)(i, j);

			Pixel32& out_pixel = out(i, j);

			float gray = 0.3 * p.r + 0.59 * p.g + 0.11 * p.b;
			out_pixel.r = gray;
			out_pixel.g = gray;
			out_pixel.b = gray;
			out_pixel.a = p.a;
		}
	}
	
	return out;
}

Image32 Image32::contrast( double contrast ) const {
	Image32 out;

	out.setSize(this->_width, this->_height);

	// calculating average luminance over all pixels

	float luminance = 0.0f;
	float avg_lum = 0.0f;

	for (int i = 0; i < this->_width; i++) {
		for (int j = 0; j < this->_height; j++) {
			Pixel32 p = (*this)(i, j);
			luminance += ((0.3 * p.r) + (0.59 * p.g) + (0.11 * p.b));
		}
	}

	avg_lum = luminance / (this->_width * this->_height);


	for (int i = 0; i < this->_width; i++) {
		for (int j = 0; j < this->_height; j++) {
			Pixel32 start = (*this)(i, j);
			
			Pixel32& output = out(i, j);

			output.r = clamp(int(((start.r - avg_lum) * contrast) + avg_lum), 0,  255);
			output.g = clamp(int(((start.g - avg_lum) * contrast) + avg_lum), 0, 255);
			output.b = clamp(int(((start.b - avg_lum) * contrast) + avg_lum), 0, 255);
			output.a = start.a;
		}
	} 
	return out;
}

Image32 Image32::saturate( double saturation ) const {
	Image32 output;
	
	output.setSize(this->_width, this->_height);

	for (int i = 0; i < this->_width; i++) {
		for (int j = 0; j < this->_height; j++) {
			Pixel32 p = (*this)(i, j);
			
			Pixel32& op = output(i, j);

			float lumin = 0.3 * p.r + 0.59 * p.g + 0.11 * p.b;

			
			op.r = clamp(int(((p.r - lumin) * saturation) + lumin), 0, 255);
			op.g = clamp(int(((p.g - lumin) * saturation) + lumin), 0, 255);;
			op.b = clamp(int(((p.b - lumin) * saturation) + lumin), 0, 255);;
			op.a = p.a;
		}
	} 


	return output;
}

Image32 Image32::quantize( int bits ) const {
	Image32 outie;
	
	outie.setSize(this->_width, this->_height);

	int range = 1 << bits;

	int step = 256 / range - 1;


	for (int i = 0; i < this->_width; i++) {
		for (int j = 0; j < this->_height; j++) {

			Pixel32 p = (*this)(i, j);

			Pixel32& op = outie(i, j);

			op.r = (p.r / step) * step;
			op.g = (p.g / step) * step;
			op.b = (p.b / step) * step;
			op.a = p.a;
		}
	}

	return outie;
}

Image32 Image32::randomDither( int bits ) const
{
	//////////////////////////////
	// Do random dithering here //
	//////////////////////////////
	WARN( "method undefined" );
	return Image32();
}

Image32 Image32::orderedDither2X2( int bits ) const
{
	///////////////////////////////
	// Do ordered dithering here //
	///////////////////////////////
	WARN( "method undefined" );
	return Image32();
}

Image32 Image32::floydSteinbergDither( int bits ) const
{
	///////////////////////////////////////
	// Do Floyd-Steinberg dithering here //
	///////////////////////////////////////
	WARN( "method undefined" );
	return Image32();
}

Image32 Image32::blur3X3( void ) const
{
	//////////////////////
	// Do blurring here //
	//////////////////////
	WARN( "method undefined" );
	return Image32();
}

Image32 Image32::edgeDetect3X3( void ) const
{
	////////////////////////////
	// Do edge detection here //
	////////////////////////////
	WARN( "method undefined" );
	return Image32();
}

Image32 Image32::scaleNearest( double scaleFactor ) const
{
	/////////////////////////////////////////////////
	// Do scaling with nearest-point sampling here //
	/////////////////////////////////////////////////
	WARN( "method undefined" );
	return Image32();
}

Image32 Image32::scaleBilinear( double scaleFactor ) const
{
	////////////////////////////////////////////
	// Do scaling with bilinear sampling here //
	////////////////////////////////////////////
	WARN( "method undefined" );
	return Image32();
}

Image32 Image32::scaleGaussian( double scaleFactor ) const
{
	////////////////////////////////////////////
	// Do scaling with Gaussian sampling here //
	////////////////////////////////////////////
	WARN( "method undefined" );
	return Image32();
}

Image32 Image32::rotateNearest( double angle ) const
{
	//////////////////////////////////////////////////
	// Do rotation with nearest-point sampling here //
	//////////////////////////////////////////////////
	WARN( "method undefined" );
	return Image32();
}

Image32 Image32::rotateBilinear( double angle ) const
{
	/////////////////////////////////////////////
	// Do rotation with bilinear sampling here //
	/////////////////////////////////////////////
	WARN( "method undefined" );
	return Image32();
}

Image32 Image32::rotateGaussian( double angle ) const
{
	/////////////////////////////////////////////
	// Do rotation with Gaussian sampling here //
	/////////////////////////////////////////////
	WARN( "method undefined" );
	return Image32();
}

void Image32::setAlpha( const Image32& matte )
{
	///////////////////////////
	// Set alpha values here //
	///////////////////////////
	WARN( "method undefined" );
}

Image32 Image32::composite( const Image32& overlay ) const
{
	/////////////////////////
	// Do compositing here //
	/////////////////////////
	WARN( "method undefined" );
	return Image32();
}

Image32 Image32::CrossDissolve( const Image32& source , const Image32& destination , double blendWeight )
{
	////////////////////////////
	// Do cross-dissolve here //
	////////////////////////////
	WARN( "method undefined" );
	return Image32();
}

Image32 Image32::warp( const OrientedLineSegmentPairs& olsp ) const
{
	/////////////////////
	// Do warping here //
	/////////////////////
	WARN( "method undefined" );
	return Image32();
}

Image32 Image32::funFilter( void ) const
{
	////////////////////////////
	// Do the fun-filter here //
	////////////////////////////
	WARN( "method undefined" );
	return Image32();
}

Image32 Image32::crop( int x1 , int y1 , int x2 , int y2 ) const
{
	//////////////////////
	// Do cropping here //
	//////////////////////
	WARN( "method undefined" );
	return Image32();
}

Pixel32 Image32::nearestSample( Point2D p ) const
{
	//////////////////////////////
	// Do nearest sampling here //
	//////////////////////////////
	WARN( "method undefined" );
	return Pixel32();
}

Pixel32 Image32::bilinearSample( Point2D p ) const
{
	///////////////////////////////
	// Do bilinear sampling here //
	///////////////////////////////
	WARN( "method undefined" );
	return Pixel32();
}

Pixel32 Image32::gaussianSample( Point2D p , double variance , double radius ) const
{
	///////////////////////////////
	// Do Gaussian sampling here //
	///////////////////////////////
	WARN( "method undefined" );
	return Pixel32();
}
