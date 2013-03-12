// Vector Operations Implemented:
//		=, +, -, *, / (on vectors and scalars)
//		Cross			Cross product vector with op
//		Dot				Dot product vector with op
//		Dist (op)		Distance from vector to op
//		DistSq			Distance^2 from vector to op
//		Length ()		Length of vector
//		Normalize ()	Normalizes vector
//

#include <math.h>

struct Vector2DC;
struct Vector2DI;

#define VNAME		2DC
#define VTYPE		unsigned char

// Vector2DC Code Definition
struct Vector2DC
{
	// Constructors/Destructors
	Vector2DC();
	Vector2DC (VTYPE xa, VTYPE ya);
	Vector2DC (Vector2DC &op);
	Vector2DC (Vector2DI &op);
//	Vector2DC (Vector2DI &op) {x=(VTYPE) op.x; y=(VTYPE) op.y;}
//	Vector2DC (Vector2DF &op) {x=(VTYPE) op.x; y=(VTYPE) op.y;}
//	Vector2DC (Vector3DC &op) {x=(VTYPE) op.x; y=(VTYPE) op.y;}
//	Vector2DC (Vector3DI &op) {x=(VTYPE) op.x; y=(VTYPE) op.y;}
//	Vector2DC (Vector3DF &op) {x=(VTYPE) op.x; y=(VTYPE) op.y;}
//	Vector2DC (Vector4DF &op) {x=(VTYPE) op.x; y=(VTYPE) op.y;}

	// Member Functions
//	Vector2DC operator= (Vector2DC &op) {x=(VTYPE) op.x; y=(VTYPE) op.y; return *this;}
//	Vector2DC operator= (Vector2DI &op) {x=(VTYPE) op.x; y=(VTYPE) op.y; return *this;}
//	Vector2DC operator= (Vector2DF &op) {x=(VTYPE) op.x; y=(VTYPE) op.y; return *this;}
//	Vector2DC operator= (Vector3DC &op) {x=(VTYPE) op.x; y=(VTYPE) op.y; return *this;}
//	Vector2DC operator= (Vector3DI &op) {x=(VTYPE) op.x; y=(VTYPE) op.y; return *this;}
//	Vector2DC operator= (Vector3DF &op) {x=(VTYPE) op.x; y=(VTYPE) op.y; return *this;}
//	Vector2DC operator= (Vector4DF &op) {x=(VTYPE) op.x; y=(VTYPE) op.y; return *this;}

//	Vector2DC operator+= (Vector2DC &op) {x+=(VTYPE) op.x; y+=(VTYPE) op.y; return *this;}
//	Vector2DC operator+= (Vector2DI &op) {x+=(VTYPE) op.x; y+=(VTYPE) op.y; return *this;}
//	Vector2DC operator+= (Vector2DF &op) {x+=(VTYPE) op.x; y+=(VTYPE) op.y; return *this;}
//	Vector2DC operator+= (Vector3DC &op) {x+=(VTYPE) op.x; y+=(VTYPE) op.y; return *this;}
//	Vector2DC operator+= (Vector3DI &op) {x+=(VTYPE) op.x; y+=(VTYPE) op.y; return *this;}
//	Vector2DC operator+= (Vector3DF &op) {x+=(VTYPE) op.x; y+=(VTYPE) op.y; return *this;}
//	Vector2DC operator+= (Vector4DF &op) {x+=(VTYPE) op.x; y+=(VTYPE) op.y; return *this;}
	
//	Vector2DC operator-= (Vector2DC &op) {x-=(VTYPE) op.x; y-=(VTYPE) op.y; return *this;}
//	Vector2DC operator-= (Vector2DI &op) {x-=(VTYPE) op.x; y-=(VTYPE) op.y; return *this;}
//	Vector2DC operator-= (Vector2DF &op) {x-=(VTYPE) op.x; y-=(VTYPE) op.y; return *this;}
//	Vector2DC operator-= (Vector3DC &op) {x-=(VTYPE) op.x; y-=(VTYPE) op.y; return *this;}
//	Vector2DC operator-= (Vector3DI &op) {x-=(VTYPE) op.x; y-=(VTYPE) op.y; return *this;}
//	Vector2DC operator-= (Vector3DF &op) {x-=(VTYPE) op.x; y-=(VTYPE) op.y; return *this;}
//	Vector2DC operator-= (Vector4DF &op) {x-=(VTYPE) op.x; y-=(VTYPE) op.y; return *this;}
	
//	Vector2DC operator*= (Vector2DC &op) {x*=(VTYPE) op.x; y*=(VTYPE) op.y; return *this;}
//	Vector2DC operator*= (Vector2DI &op) {x*=(VTYPE) op.x; y*=(VTYPE) op.y; return *this;}
//	Vector2DC operator*= (Vector2DF &op) {x*=(VTYPE) op.x; y*=(VTYPE) op.y; return *this;}
//	Vector2DC operator*= (Vector3DC &op) {x*=(VTYPE) op.x; y*=(VTYPE) op.y; return *this;}
//	Vector2DC operator*= (Vector3DI &op) {x*=(VTYPE) op.x; y*=(VTYPE) op.y; return *this;}
//	Vector2DC operator*= (Vector3DF &op) {x*=(VTYPE) op.x; y*=(VTYPE) op.y; return *this;}
//	Vector2DC operator*= (Vector4DF &op) {x*=(VTYPE) op.x; y*=(VTYPE) op.y; return *this;}
	
//	Vector2DC operator/= (Vector2DC &op) {x/=(VTYPE) op.x; y/=(VTYPE) op.y; return *this;}
//	Vector2DC operator/= (Vector2DI &op) {x/=(VTYPE) op.x; y/=(VTYPE) op.y; return *this;}
//	Vector2DC operator/= (Vector2DF &op) {x/=(VTYPE) op.x; y/=(VTYPE) op.y; return *this;}
//	Vector2DC operator/= (Vector3DC &op) {x/=(VTYPE) op.x; y/=(VTYPE) op.y; return *this;}
//	Vector2DC operator/= (Vector3DI &op) {x/=(VTYPE) op.x; y/=(VTYPE) op.y; return *this;}
//	Vector2DC operator/= (Vector3DF &op) {x/=(VTYPE) op.x; y/=(VTYPE) op.y; return *this;}
//	Vector2DC operator/= (Vector4DF &op) {x/=(VTYPE) op.x; y/=(VTYPE) op.y; return *this;}
	
	// Note: Cross product does not exist for 2D vectors (only 3D)
	double Dot(Vector2DC &v)			{double dot; dot = (double) x*v.x + (double) y*v.y; return dot;}
//	double Dot(Vector2DI &v)			{double dot; dot = (double) x*v.x + (double) y*v.y; return dot;}
//	double Dot(Vector2DF &v)			{double dot; dot = (double) x*v.x + (double) y*v.y; return dot;}
	
	double Dist (Vector2DC &v)		{ double distsq = DistSq (v); if (distsq!=0) return sqrt(distsq); return 0.0;}
//	double Dist (Vector2DI &v)		{ double distsq = DistSq (v); if (distsq!=0) return sqrt(distsq); return 0.0;}
//	double Dist (Vector2DF &v)		{ double distsq = DistSq (v); if (distsq!=0) return sqrt(distsq); return 0.0;}
//	double Dist (Vector3DC &v)		{ double distsq = DistSq (v); if (distsq!=0) return sqrt(distsq); return 0.0;}
//	double Dist (Vector3DI &v)		{ double distsq = DistSq (v); if (distsq!=0) return sqrt(distsq); return 0.0;}
//	double Dist (Vector3DF &v)		{ double distsq = DistSq (v); if (distsq!=0) return sqrt(distsq); return 0.0;}
//	double Dist (Vector4DF &v)		{ double distsq = DistSq (v); if (distsq!=0) return sqrt(distsq); return 0.0;}
	double DistSq (Vector2DC &v)		{ double a,b; a = (double) x - (double) v.x; b = (double) y - (double) v.y; return (a*a + b*b);}
//	double DistSq (Vector2DI &v)		{ double a,b; a = (double) x - (double) v.x; b = (double) y - (double) v.y; return (a*a + b*b);}
//	double DistSq (Vector2DF &v)		{ double a,b; a = (double) x - (double) v.x; b = (double) y - (double) v.y; return (a*a + b*b);}
//	double DistSq (Vector3DC &v)		{ double a,b; a = (double) x - (double) v.x; b = (double) y - (double) v.y; return (a*a + b*b);}
//	double DistSq (Vector3DI &v)		{ double a,b; a = (double) x - (double) v.x; b = (double) y - (double) v.y; return (a*a + b*b);}
//	double DistSq (Vector3DF &v)		{ double a,b; a = (double) x - (double) v.x; b = (double) y - (double) v.y; return (a*a + b*b);}
//	double DistSq (Vector4DF &v)		{ double a,b; a = (double) x - (double) v.x; b = (double) y - (double) v.y; return (a*a + b*b);}
	
	Vector2DC& Normalize (void) {
		double n = (double) x*x + (double) y*y;
		if (n!=0.0) {
			n = sqrt(n);
			x = (VTYPE) (((double) x*255)/n);
			y = (VTYPE) (((double) y*255)/n);
		}
		return *this;
	}
	double Length (void) { double n; n = (double) x*x + (double) y*y; if (n != 0.0) return sqrt(n); return 0.0; }
	VTYPE *Data (void)			{return &x;}
	
	VTYPE x,y;
};

#undef VTYPE
#undef VNAME

// Vector2DI Code Definition

#define VNAME		2DI
#define VTYPE		int

struct Vector2DI
{
	// Constructors/Destructors
	Vector2DI() {x=0; y=0;}
	Vector2DI (const VTYPE xa, const VTYPE ya) {x=xa; y=ya;}
	Vector2DI (const Vector2DC &op) {x=(VTYPE) op.x; y=(VTYPE) op.y;}
//	Vector2DI (const Vector2DI &op) {x=(VTYPE) op.x; y=(VTYPE) op.y;}
//	Vector2DI (const Vector2DF &op) {x=(VTYPE) op.x; y=(VTYPE) op.y;}
//	Vector2DI (const Vector3DC &op) {x=(VTYPE) op.x; y=(VTYPE) op.y;}
//	Vector2DI (const Vector3DI &op) {x=(VTYPE) op.x; y=(VTYPE) op.y;}
//	Vector2DI (const Vector3DF &op) {x=(VTYPE) op.x; y=(VTYPE) op.y;}
//	Vector2DI (const Vector4DF &op) {x=(VTYPE) op.x; y=(VTYPE) op.y;}

	VTYPE x,y;
};

//// Member Functions
// Vector2DI &Vector2DI::operator= (const Vector2DC &op) {x=(VTYPE) op.x; y=(VTYPE) op.y; return *this;}
// Vector2DI &Vector2DI::operator= (const Vector2DI &op) {x=(VTYPE) op.x; y=(VTYPE) op.y; return *this;}
// Vector2DI &Vector2DI::operator= (const Vector2DF &op) {x=(VTYPE) op.x; y=(VTYPE) op.y; return *this;}
// Vector2DI &Vector2DI::operator= (const Vector3DC &op) {x=(VTYPE) op.x; y=(VTYPE) op.y; return *this;}
// Vector2DI &Vector2DI::operator= (const Vector3DI &op) {x=(VTYPE) op.x; y=(VTYPE) op.y; return *this;}
// Vector2DI &Vector2DI::operator= (const Vector3DF &op) {x=(VTYPE) op.x; y=(VTYPE) op.y; return *this;}
// Vector2DI &Vector2DI::operator= (const Vector4DF &op) {x=(VTYPE) op.x; y=(VTYPE) op.y; return *this;}	
//	
// Vector2DI &Vector2DI::operator+= (const Vector2DC &op) {x+=(VTYPE) op.x; y+=(VTYPE) op.y; return *this;}
// Vector2DI &Vector2DI::operator+= (const Vector2DI &op) {x+=(VTYPE) op.x; y+=(VTYPE) op.y; return *this;}
// Vector2DI &Vector2DI::operator+= (const Vector2DF &op) {x+=(VTYPE) op.x; y+=(VTYPE) op.y; return *this;}
// Vector2DI &Vector2DI::operator+= (const Vector3DC &op) {x+=(VTYPE) op.x; y+=(VTYPE) op.y; return *this;}
// Vector2DI &Vector2DI::operator+= (const Vector3DI &op) {x+=(VTYPE) op.x; y+=(VTYPE) op.y; return *this;}
// Vector2DI &Vector2DI::operator+= (const Vector3DF &op) {x+=(VTYPE) op.x; y+=(VTYPE) op.y; return *this;}
// Vector2DI &Vector2DI::operator+= (const Vector4DF &op) {x+=(VTYPE) op.x; y+=(VTYPE) op.y; return *this;}
//
// Vector2DI &Vector2DI::operator-= (const Vector2DC &op) {x-=(VTYPE) op.x; y-=(VTYPE) op.y; return *this;}
// Vector2DI &Vector2DI::operator-= (const Vector2DI &op) {x-=(VTYPE) op.x; y-=(VTYPE) op.y; return *this;}
// Vector2DI &Vector2DI::operator-= (const Vector2DF &op) {x-=(VTYPE) op.x; y-=(VTYPE) op.y; return *this;}
// Vector2DI &Vector2DI::operator-= (const Vector3DC &op) {x-=(VTYPE) op.x; y-=(VTYPE) op.y; return *this;}
// Vector2DI &Vector2DI::operator-= (const Vector3DI &op) {x-=(VTYPE) op.x; y-=(VTYPE) op.y; return *this;}
// Vector2DI &Vector2DI::operator-= (const Vector3DF &op) {x-=(VTYPE) op.x; y-=(VTYPE) op.y; return *this;}
// Vector2DI &Vector2DI::operator-= (const Vector4DF &op) {x-=(VTYPE) op.x; y-=(VTYPE) op.y; return *this;}
//	
// Vector2DI &Vector2DI::operator*= (const Vector2DC &op) {x*=(VTYPE) op.x; y*=(VTYPE) op.y; return *this;}
// Vector2DI &Vector2DI::operator*= (const Vector2DI &op) {x*=(VTYPE) op.x; y*=(VTYPE) op.y; return *this;}
// Vector2DI &Vector2DI::operator*= (const Vector2DF &op) {x*=(VTYPE) op.x; y*=(VTYPE) op.y; return *this;}
// Vector2DI &Vector2DI::operator*= (const Vector3DC &op) {x*=(VTYPE) op.x; y*=(VTYPE) op.y; return *this;}
// Vector2DI &Vector2DI::operator*= (const Vector3DI &op) {x*=(VTYPE) op.x; y*=(VTYPE) op.y; return *this;}
// Vector2DI &Vector2DI::operator*= (const Vector3DF &op) {x*=(VTYPE) op.x; y*=(VTYPE) op.y; return *this;}
// Vector2DI &Vector2DI::operator*= (const Vector4DF &op) {x*=(VTYPE) op.x; y*=(VTYPE) op.y; return *this;}
//
// Vector2DI &Vector2DI::operator/= (const Vector2DC &op) {x/=(VTYPE) op.x; y/=(VTYPE) op.y; return *this;}
// Vector2DI &Vector2DI::operator/= (const Vector2DI &op) {x/=(VTYPE) op.x; y/=(VTYPE) op.y; return *this;}
// Vector2DI &Vector2DI::operator/= (const Vector2DF &op) {x/=(VTYPE) op.x; y/=(VTYPE) op.y; return *this;}
// Vector2DI &Vector2DI::operator/= (const Vector3DC &op) {x/=(VTYPE) op.x; y/=(VTYPE) op.y; return *this;}
// Vector2DI &Vector2DI::operator/= (const Vector3DI &op) {x/=(VTYPE) op.x; y/=(VTYPE) op.y; return *this;}
// Vector2DI &Vector2DI::operator/= (const Vector3DF &op) {x/=(VTYPE) op.x; y/=(VTYPE) op.y; return *this;}
// Vector2DI &Vector2DI::operator/= (const Vector4DF &op) {x/=(VTYPE) op.x; y/=(VTYPE) op.y; return *this;}
//
//// Note: Cross product does not exist for 2D vectors (only 3D)
//		
// double Vector2DI::Dot(const Vector2DC &v)			{double dot; dot = (double) x*v.x + (double) y*v.y; return dot;}
// double Vector2DI::Dot(const Vector2DI &v)			{double dot; dot = (double) x*v.x + (double) y*v.y; return dot;}
// double Vector2DI::Dot(const Vector2DF &v)			{double dot; dot = (double) x*v.x + (double) y*v.y; return dot;}
//
// double Vector2DI::Dist (const Vector2DC &v)		{ double distsq = DistSq (v); if (distsq!=0) return sqrt(distsq); return 0.0;}
// double Vector2DI::Dist (const Vector2DI &v)		{ double distsq = DistSq (v); if (distsq!=0) return sqrt(distsq); return 0.0;}
// double Vector2DI::Dist (const Vector2DF &v)		{ double distsq = DistSq (v); if (distsq!=0) return sqrt(distsq); return 0.0;}
// double Vector2DI::Dist (const Vector3DC &v)		{ double distsq = DistSq (v); if (distsq!=0) return sqrt(distsq); return 0.0;}
// double Vector2DI::Dist (const Vector3DI &v)		{ double distsq = DistSq (v); if (distsq!=0) return sqrt(distsq); return 0.0;}
// double Vector2DI::Dist (const Vector3DF &v)		{ double distsq = DistSq (v); if (distsq!=0) return sqrt(distsq); return 0.0;}
// double Vector2DI::Dist (const Vector4DF &v)		{ double distsq = DistSq (v); if (distsq!=0) return sqrt(distsq); return 0.0;}
// double Vector2DI::DistSq (const Vector2DC &v)		{ double a,b; a = (double) x - (double) v.x; b = (double) y - (double) v.y; return (a*a + b*b);}
// double Vector2DI::DistSq (const Vector2DI &v)		{ double a,b; a = (double) x - (double) v.x; b = (double) y - (double) v.y; return (a*a + b*b);}
// double Vector2DI::DistSq (const Vector2DF &v)		{ double a,b; a = (double) x - (double) v.x; b = (double) y - (double) v.y; return (a*a + b*b);}
// double Vector2DI::DistSq (const Vector3DC &v)		{ double a,b; a = (double) x - (double) v.x; b = (double) y - (double) v.y; return (a*a + b*b);}
// double Vector2DI::DistSq (const Vector3DI &v)		{ double a,b; a = (double) x - (double) v.x; b = (double) y - (double) v.y; return (a*a + b*b);}
// double Vector2DI::DistSq (const Vector3DF &v)		{ double a,b; a = (double) x - (double) v.x; b = (double) y - (double) v.y; return (a*a + b*b);}
// double Vector2DI::DistSq (const Vector4DF &v)		{ double a,b; a = (double) x - (double) v.x; b = (double) y - (double) v.y; return (a*a + b*b);}
//
// Vector2DI &Vector2DI::Normalize (void) {
//	double n = (double) x*x + (double) y*y;
//	if (n!=0.0) {
//		n = sqrt(n);
//		x = (VTYPE) (((double) x*255)/n);
//		y = (VTYPE) (((double) y*255)/n);				
//	}
//	return *this;
//}
// double Vector2DI::Length (void) { double n; n = (double) x*x + (double) y*y; if (n != 0.0) return sqrt(n); return 0.0; }
// VTYPE *Vector2DI::Data (void)			{return &x;}
//
//#undef VTYPE
//#undef VNAME
//
//// Vector2DF Code Definition
//
//#define VNAME		2DF
//#define VTYPE		float
//
//// Constructors/Destructors
// Vector2DF::Vector2DF() {x=0; y=0;}
// Vector2DF::~Vector2DF() {}
// Vector2DF::Vector2DF (const VTYPE xa, const VTYPE ya) {x=xa; y=ya;}
// Vector2DF::Vector2DF (const Vector2DC &op) {x=(VTYPE) op.x; y=(VTYPE) op.y;}
// Vector2DF::Vector2DF (const Vector2DI &op) {x=(VTYPE) op.x; y=(VTYPE) op.y;}
// Vector2DF::Vector2DF (const Vector2DF &op) {x=(VTYPE) op.x; y=(VTYPE) op.y;}
// Vector2DF::Vector2DF (const Vector3DC &op) {x=(VTYPE) op.x; y=(VTYPE) op.y;}
// Vector2DF::Vector2DF (const Vector3DI &op) {x=(VTYPE) op.x; y=(VTYPE) op.y;}
// Vector2DF::Vector2DF (const Vector3DF &op) {x=(VTYPE) op.x; y=(VTYPE) op.y;}
// Vector2DF::Vector2DF (const Vector4DF &op) {x=(VTYPE) op.x; y=(VTYPE) op.y;}
//
// Vector2DF &Vector2DF::Set (const float xa, const float ya) { x = (VTYPE) xa; y = (VTYPE) ya; return *this; }
//
//// Member Functions
// Vector2DF &Vector2DF::operator= (const Vector2DC &op) {x=(VTYPE) op.x; y=(VTYPE) op.y; return *this;}
// Vector2DF &Vector2DF::operator= (const Vector2DI &op) {x=(VTYPE) op.x; y=(VTYPE) op.y; return *this;}
// Vector2DF &Vector2DF::operator= (const Vector2DF &op) {x=(VTYPE) op.x; y=(VTYPE) op.y; return *this;}
// Vector2DF &Vector2DF::operator= (const Vector3DC &op) {x=(VTYPE) op.x; y=(VTYPE) op.y; return *this;}
// Vector2DF &Vector2DF::operator= (const Vector3DI &op) {x=(VTYPE) op.x; y=(VTYPE) op.y; return *this;}
// Vector2DF &Vector2DF::operator= (const Vector3DF &op) {x=(VTYPE) op.x; y=(VTYPE) op.y; return *this;}
// Vector2DF &Vector2DF::operator= (const Vector4DF &op) {x=(VTYPE) op.x; y=(VTYPE) op.y; return *this;}	
//	
// Vector2DF &Vector2DF::operator+= (const Vector2DC &op) {x+=(VTYPE) op.x; y+=(VTYPE) op.y; return *this;}
// Vector2DF &Vector2DF::operator+= (const Vector2DI &op) {x+=(VTYPE) op.x; y+=(VTYPE) op.y; return *this;}
// Vector2DF &Vector2DF::operator+= (const Vector2DF &op) {x+=(VTYPE) op.x; y+=(VTYPE) op.y; return *this;}
// Vector2DF &Vector2DF::operator+= (const Vector3DC &op) {x+=(VTYPE) op.x; y+=(VTYPE) op.y; return *this;}
// Vector2DF &Vector2DF::operator+= (const Vector3DI &op) {x+=(VTYPE) op.x; y+=(VTYPE) op.y; return *this;}
// Vector2DF &Vector2DF::operator+= (const Vector3DF &op) {x+=(VTYPE) op.x; y+=(VTYPE) op.y; return *this;}
// Vector2DF &Vector2DF::operator+= (const Vector4DF &op) {x+=(VTYPE) op.x; y+=(VTYPE) op.y; return *this;}
//
// Vector2DF &Vector2DF::operator-= (const Vector2DC &op) {x-=(VTYPE) op.x; y-=(VTYPE) op.y; return *this;}
// Vector2DF &Vector2DF::operator-= (const Vector2DI &op) {x-=(VTYPE) op.x; y-=(VTYPE) op.y; return *this;}
// Vector2DF &Vector2DF::operator-= (const Vector2DF &op) {x-=(VTYPE) op.x; y-=(VTYPE) op.y; return *this;}
// Vector2DF &Vector2DF::operator-= (const Vector3DC &op) {x-=(VTYPE) op.x; y-=(VTYPE) op.y; return *this;}
// Vector2DF &Vector2DF::operator-= (const Vector3DI &op) {x-=(VTYPE) op.x; y-=(VTYPE) op.y; return *this;}
// Vector2DF &Vector2DF::operator-= (const Vector3DF &op) {x-=(VTYPE) op.x; y-=(VTYPE) op.y; return *this;}
// Vector2DF &Vector2DF::operator-= (const Vector4DF &op) {x-=(VTYPE) op.x; y-=(VTYPE) op.y; return *this;}
//	
// Vector2DF &Vector2DF::operator*= (const Vector2DC &op) {x*=(VTYPE) op.x; y*=(VTYPE) op.y; return *this;}
// Vector2DF &Vector2DF::operator*= (const Vector2DI &op) {x*=(VTYPE) op.x; y*=(VTYPE) op.y; return *this;}
// Vector2DF &Vector2DF::operator*= (const Vector2DF &op) {x*=(VTYPE) op.x; y*=(VTYPE) op.y; return *this;}
// Vector2DF &Vector2DF::operator*= (const Vector3DC &op) {x*=(VTYPE) op.x; y*=(VTYPE) op.y; return *this;}
// Vector2DF &Vector2DF::operator*= (const Vector3DI &op) {x*=(VTYPE) op.x; y*=(VTYPE) op.y; return *this;}
// Vector2DF &Vector2DF::operator*= (const Vector3DF &op) {x*=(VTYPE) op.x; y*=(VTYPE) op.y; return *this;}
// Vector2DF &Vector2DF::operator*= (const Vector4DF &op) {x*=(VTYPE) op.x; y*=(VTYPE) op.y; return *this;}
//
// Vector2DF &Vector2DF::operator/= (const Vector2DC &op) {x/=(VTYPE) op.x; y/=(VTYPE) op.y; return *this;}
// Vector2DF &Vector2DF::operator/= (const Vector2DI &op) {x/=(VTYPE) op.x; y/=(VTYPE) op.y; return *this;}
// Vector2DF &Vector2DF::operator/= (const Vector2DF &op) {x/=(VTYPE) op.x; y/=(VTYPE) op.y; return *this;}
// Vector2DF &Vector2DF::operator/= (const Vector3DC &op) {x/=(VTYPE) op.x; y/=(VTYPE) op.y; return *this;}
// Vector2DF &Vector2DF::operator/= (const Vector3DI &op) {x/=(VTYPE) op.x; y/=(VTYPE) op.y; return *this;}
// Vector2DF &Vector2DF::operator/= (const Vector3DF &op) {x/=(VTYPE) op.x; y/=(VTYPE) op.y; return *this;}
// Vector2DF &Vector2DF::operator/= (const Vector4DF &op) {x/=(VTYPE) op.x; y/=(VTYPE) op.y; return *this;}
//
//// Note: Cross product does not exist for 2D vectors (only 3D)
//		
// double Vector2DF::Dot(const Vector2DC &v)			{double dot; dot = (double) x*v.x + (double) y*v.y; return dot;}
// double Vector2DF::Dot(const Vector2DI &v)			{double dot; dot = (double) x*v.x + (double) y*v.y; return dot;}
// double Vector2DF::Dot(const Vector2DF &v)			{double dot; dot = (double) x*v.x + (double) y*v.y; return dot;}
//
// double Vector2DF::Dist (const Vector2DC &v)		{ double distsq = DistSq (v); if (distsq!=0) return sqrt(distsq); return 0.0;}
// double Vector2DF::Dist (const Vector2DI &v)		{ double distsq = DistSq (v); if (distsq!=0) return sqrt(distsq); return 0.0;}
// double Vector2DF::Dist (const Vector2DF &v)		{ double distsq = DistSq (v); if (distsq!=0) return sqrt(distsq); return 0.0;}
// double Vector2DF::Dist (const Vector3DC &v)		{ double distsq = DistSq (v); if (distsq!=0) return sqrt(distsq); return 0.0;}
// double Vector2DF::Dist (const Vector3DI &v)		{ double distsq = DistSq (v); if (distsq!=0) return sqrt(distsq); return 0.0;}
// double Vector2DF::Dist (const Vector3DF &v)		{ double distsq = DistSq (v); if (distsq!=0) return sqrt(distsq); return 0.0;}
// double Vector2DF::Dist (const Vector4DF &v)		{ double distsq = DistSq (v); if (distsq!=0) return sqrt(distsq); return 0.0;}
// double Vector2DF::DistSq (const Vector2DC &v)		{ double a,b; a = (double) x - (double) v.x; b = (double) y - (double) v.y; return (a*a + b*b);}
// double Vector2DF::DistSq (const Vector2DI &v)		{ double a,b; a = (double) x - (double) v.x; b = (double) y - (double) v.y; return (a*a + b*b);}
// double Vector2DF::DistSq (const Vector2DF &v)		{ double a,b; a = (double) x - (double) v.x; b = (double) y - (double) v.y; return (a*a + b*b);}
// double Vector2DF::DistSq (const Vector3DC &v)		{ double a,b; a = (double) x - (double) v.x; b = (double) y - (double) v.y; return (a*a + b*b);}
// double Vector2DF::DistSq (const Vector3DI &v)		{ double a,b; a = (double) x - (double) v.x; b = (double) y - (double) v.y; return (a*a + b*b);}
// double Vector2DF::DistSq (const Vector3DF &v)		{ double a,b; a = (double) x - (double) v.x; b = (double) y - (double) v.y; return (a*a + b*b);}
// double Vector2DF::DistSq (const Vector4DF &v)		{ double a,b; a = (double) x - (double) v.x; b = (double) y - (double) v.y; return (a*a + b*b);}
//
// Vector2DF &Vector2DF::Normalize (void) {
//	VTYPE n = (VTYPE) x*x + (VTYPE) y*y;
//	if (n!=0.0) {
//		n = sqrt(n);
//		x /= n;
//		y /= n;
//	}
//	return *this;
//}
// double Vector2DF::Length (void) { double n; n = (double) x*x + (double) y*y; if (n != 0.0) return sqrt(n); return 0.0; }
// VTYPE *Vector2DF::Data (void)			{return &x;}
//
//#undef VTYPE
//#undef VNAME
//
//// Vector3DC Code Definition
//
//#define VNAME		3DC
//#define VTYPE		unsigned char
//
//// Constructors/Destructors
// Vector3DC::Vector3DC() {x=0; y=0; z=0;}
// Vector3DC::~Vector3DC() {}
// Vector3DC::Vector3DC (const VTYPE xa, const VTYPE ya, const VTYPE za) {x=xa; y=ya; z=za;}
// Vector3DC::Vector3DC (const Vector2DC &op) {x=(VTYPE) op.x; y=(VTYPE) op.y; z=(VTYPE) 0;}
// Vector3DC::Vector3DC (const Vector2DI &op) {x=(VTYPE) op.x; y=(VTYPE) op.y; z=(VTYPE) 0;}
// Vector3DC::Vector3DC (const Vector2DF &op) {x=(VTYPE) op.x; y=(VTYPE) op.y; z=(VTYPE) 0;}
// Vector3DC::Vector3DC (const Vector3DC &op) {x=(VTYPE) op.x; y=(VTYPE) op.y; z=(VTYPE) op.z;}
// Vector3DC::Vector3DC (const Vector3DI &op) {x=(VTYPE) op.x; y=(VTYPE) op.y; z=(VTYPE) op.z;}
// Vector3DC::Vector3DC (const Vector3DF &op) {x=(VTYPE) op.x; y=(VTYPE) op.y; z=(VTYPE) op.z;}
// Vector3DC::Vector3DC (const Vector4DF &op) {x=(VTYPE) op.x; y=(VTYPE) op.y; z=(VTYPE) op.z;}
//
//// Member Functions
// Vector3DC &Vector3DC::Set (const VTYPE xa, const VTYPE ya, const VTYPE za) {x=xa; y=ya; z=za; return *this;}
//
// Vector3DC &Vector3DC::operator= (const Vector2DC &op) {x=(VTYPE) op.x; y=(VTYPE) op.y; z=(VTYPE) 0; return *this;}
// Vector3DC &Vector3DC::operator= (const Vector2DI &op) {x=(VTYPE) op.x; y=(VTYPE) op.y; z=(VTYPE) 0; return *this;}
// Vector3DC &Vector3DC::operator= (const Vector2DF &op) {x=(VTYPE) op.x; y=(VTYPE) op.y; z=(VTYPE) 0; return *this;}
// Vector3DC &Vector3DC::operator=  ( const Vector3DC &op) {x=(VTYPE) op.x; y=(VTYPE) op.y; z=(VTYPE) op.z; return *this;}
// Vector3DC &Vector3DC::operator=  ( const Vector3DI &op) {x=(VTYPE) op.x; y=(VTYPE) op.y; z=(VTYPE) op.z; return *this;}
// Vector3DC &Vector3DC::operator=  ( const Vector3DF &op) {x=(VTYPE) op.x; y=(VTYPE) op.y; z=(VTYPE) op.z; return *this;}
// Vector3DC &Vector3DC::operator=  ( const Vector4DF &op) {x=(VTYPE) op.x; y=(VTYPE) op.y; z=(VTYPE) op.z; return *this;}	
//	
// Vector3DC &Vector3DC::operator+=  ( const Vector2DC &op) {x+=(VTYPE) op.x; y+=(VTYPE) op.y; return *this;}
// Vector3DC &Vector3DC::operator+=  ( const Vector2DI &op) {x+=(VTYPE) op.x; y+=(VTYPE) op.y; return *this;}
// Vector3DC &Vector3DC::operator+=  ( const Vector2DF &op) {x+=(VTYPE) op.x; y+=(VTYPE) op.y; return *this;}
// Vector3DC &Vector3DC::operator+=  ( const Vector3DC &op) {x+=(VTYPE) op.x; y+=(VTYPE) op.y; z+=(VTYPE) op.z; return *this;}
// Vector3DC &Vector3DC::operator+=  ( const Vector3DI &op) {x+=(VTYPE) op.x; y+=(VTYPE) op.y; z+=(VTYPE) op.z; return *this;}
// Vector3DC &Vector3DC::operator+=  ( const Vector3DF &op) {x+=(VTYPE) op.x; y+=(VTYPE) op.y; z+=(VTYPE) op.z; return *this;}
// Vector3DC &Vector3DC::operator+=  ( const Vector4DF &op) {x+=(VTYPE) op.x; y+=(VTYPE) op.y; z+=(VTYPE) op.z; return *this;}
//
// Vector3DC &Vector3DC::operator-=  ( const Vector2DC &op) {x-=(VTYPE) op.x; y-=(VTYPE) op.y; return *this;}
// Vector3DC &Vector3DC::operator-=  ( const Vector2DI &op) {x-=(VTYPE) op.x; y-=(VTYPE) op.y; return *this;}
// Vector3DC &Vector3DC::operator-=  ( const Vector2DF &op) {x-=(VTYPE) op.x; y-=(VTYPE) op.y; return *this;}
// Vector3DC &Vector3DC::operator-=  ( const Vector3DC &op) {x-=(VTYPE) op.x; y-=(VTYPE) op.y; z-=(VTYPE) op.z; return *this;}
// Vector3DC &Vector3DC::operator-=  ( const Vector3DI &op) {x-=(VTYPE) op.x; y-=(VTYPE) op.y; z-=(VTYPE) op.z; return *this;}
// Vector3DC &Vector3DC::operator-=  ( const Vector3DF &op) {x-=(VTYPE) op.x; y-=(VTYPE) op.y; z-=(VTYPE) op.z; return *this;}
// Vector3DC &Vector3DC::operator-=  ( const Vector4DF &op) {x-=(VTYPE) op.x; y-=(VTYPE) op.y; z-=(VTYPE) op.z; return *this;}
//	
// Vector3DC &Vector3DC::operator*=  ( const Vector2DC &op) {x*=(VTYPE) op.x; y*=(VTYPE) op.y; return *this;}
// Vector3DC &Vector3DC::operator*=  ( const Vector2DI &op) {x*=(VTYPE) op.x; y*=(VTYPE) op.y; return *this;}
// Vector3DC &Vector3DC::operator*=  ( const Vector2DF &op) {x*=(VTYPE) op.x; y*=(VTYPE) op.y; return *this;}
// Vector3DC &Vector3DC::operator*=  ( const Vector3DC &op) {x*=(VTYPE) op.x; y*=(VTYPE) op.y; z*=(VTYPE) op.z; return *this;}
// Vector3DC &Vector3DC::operator*=  ( const Vector3DI &op) {x*=(VTYPE) op.x; y*=(VTYPE) op.y; z*=(VTYPE) op.z; return *this;}
// Vector3DC &Vector3DC::operator*=  ( const Vector3DF &op) {x*=(VTYPE) op.x; y*=(VTYPE) op.y; z*=(VTYPE) op.z; return *this;}
// Vector3DC &Vector3DC::operator*=  ( const Vector4DF &op) {x*=(VTYPE) op.x; y*=(VTYPE) op.y; z*=(VTYPE) op.z; return *this;}
//
// Vector3DC &Vector3DC::operator/=  ( const Vector2DC &op) {x/=(VTYPE) op.x; y/=(VTYPE) op.y; return *this;}
// Vector3DC &Vector3DC::operator/=  ( const Vector2DI &op) {x/=(VTYPE) op.x; y/=(VTYPE) op.y; return *this;}
// Vector3DC &Vector3DC::operator/=  ( const Vector2DF &op) {x/=(VTYPE) op.x; y/=(VTYPE) op.y; return *this;}
// Vector3DC &Vector3DC::operator/=  ( const Vector3DC &op) {x/=(VTYPE) op.x; y/=(VTYPE) op.y; z/=(VTYPE) op.z; return *this;}
// Vector3DC &Vector3DC::operator/=  ( const Vector3DI &op) {x/=(VTYPE) op.x; y/=(VTYPE) op.y; z/=(VTYPE) op.z; return *this;}
// Vector3DC &Vector3DC::operator/=  ( const Vector3DF &op) {x/=(VTYPE) op.x; y/=(VTYPE) op.y; z/=(VTYPE) op.z; return *this;}
// Vector3DC &Vector3DC::operator/=  ( const Vector4DF &op) {x/=(VTYPE) op.x; y/=(VTYPE) op.y; z/=(VTYPE) op.z; return *this;}
//
// Vector3DC &Vector3DC::Cross  ( const Vector3DC &v) {double ax = x, ay = y, az = z; x = (VTYPE) (ay * (double) v.z - az * (double) v.y); y = (VTYPE) (-ax * (double) v.z + az * (double) v.x); z = (VTYPE) (ax * (double) v.y - ay * (double) v.x); return *this;}
// Vector3DC &Vector3DC::Cross  ( const Vector3DI &v) {double ax = x, ay = y, az = z; x = (VTYPE) (ay * (double) v.z - az * (double) v.y); y = (VTYPE) (-ax * (double) v.z + az * (double) v.x); z = (VTYPE) (ax * (double) v.y - ay * (double) v.x); return *this;}
// Vector3DC &Vector3DC::Cross  ( const Vector3DF &v) {double ax = x, ay = y, az = z; x = (VTYPE) (ay * (double) v.z - az * (double) v.y); y = (VTYPE) (-ax * (double) v.z + az * (double) v.x); z = (VTYPE) (ax * (double) v.y - ay * (double) v.x); return *this;}
//
// double Vector3DC::Dot ( const Vector3DC &v)			{double dot; dot = (double) x*v.x + (double) y*v.y + (double) z*v.z; return dot;}
// double Vector3DC::Dot ( const Vector3DI &v)			{double dot; dot = (double) x*v.x + (double) y*v.y + (double) z*v.z; return dot;}
// double Vector3DC::Dot ( const Vector3DF &v)			{double dot; dot = (double) x*v.x + (double) y*v.y + (double) z*v.z; return dot;}
//
// double Vector3DC::Dist  ( const Vector2DC &v)		{ double distsq = DistSq (v); if (distsq!=0) return sqrt(distsq); return 0.0;}
// double Vector3DC::Dist  ( const Vector2DI &v)		{ double distsq = DistSq (v); if (distsq!=0) return sqrt(distsq); return 0.0;}
// double Vector3DC::Dist  ( const Vector2DF &v)		{ double distsq = DistSq (v); if (distsq!=0) return sqrt(distsq); return 0.0;}
// double Vector3DC::Dist  ( const Vector3DC &v)		{ double distsq = DistSq (v); if (distsq!=0) return sqrt(distsq); return 0.0;}
// double Vector3DC::Dist  ( const Vector3DI &v)		{ double distsq = DistSq (v); if (distsq!=0) return sqrt(distsq); return 0.0;}
// double Vector3DC::Dist  ( const Vector3DF &v)		{ double distsq = DistSq (v); if (distsq!=0) return sqrt(distsq); return 0.0;}
// double Vector3DC::Dist  ( const Vector4DF &v)		{ double distsq = DistSq (v); if (distsq!=0) return sqrt(distsq); return 0.0;}
// double Vector3DC::DistSq  ( const Vector2DC &v)		{ double a,b,c; a = (double) x - (double) v.x; b = (double) y - (double) v.y; c = (double) z; return (a*a + b*b + c*c);}
// double Vector3DC::DistSq  ( const Vector2DI &v)		{ double a,b,c; a = (double) x - (double) v.x; b = (double) y - (double) v.y; c = (double) z; return (a*a + b*b + c*c);}
// double Vector3DC::DistSq  ( const Vector2DF &v)		{ double a,b,c; a = (double) x - (double) v.x; b = (double) y - (double) v.y; c = (double) z; return (a*a + b*b + c*c);}
// double Vector3DC::DistSq  ( const Vector3DC &v)		{ double a,b,c; a = (double) x - (double) v.x; b = (double) y - (double) v.y; c = (double) z - (double) v.z; return (a*a + b*b + c*c);}
// double Vector3DC::DistSq  ( const Vector3DI &v)		{ double a,b,c; a = (double) x - (double) v.x; b = (double) y - (double) v.y; c = (double) z - (double) v.z; return (a*a + b*b + c*c);}
// double Vector3DC::DistSq  ( const Vector3DF &v)		{ double a,b,c; a = (double) x - (double) v.x; b = (double) y - (double) v.y; c = (double) z - (double) v.z; return (a*a + b*b + c*c);}
// double Vector3DC::DistSq  ( const Vector4DF &v)		{ double a,b,c; a = (double) x - (double) v.x; b = (double) y - (double) v.y; c = (double) z - (double) v.z; return (a*a + b*b + c*c);}
//
// Vector3DC &Vector3DC::Normalize (void) {
//	double n = (double) x*x + (double) y*y + (double) z*z;
//	if (n!=0.0) {
//		n = sqrt(n);
//		x = (VTYPE) (((double) x*255)/n);
//		y = (VTYPE) (((double) y*255)/n);
//		z = (VTYPE) (((double) z*255)/n);
//	}
//	return *this;
//}
// double Vector3DC::Length (void) { double n; n = (double) x*x + (double) y*y + (double) z*z; if (n != 0.0) return sqrt(n); return 0.0; }
// VTYPE *Vector3DC::Data (void)			{return &x;}
//
//#undef VTYPE
//#undef VNAME
//
//// Vector3DI Code Definition
//
//#define VNAME		3DI
//#define VTYPE		int
//
//// Constructors/Destructors
// Vector3DI::Vector3DI() {x=0; y=0; z=0;}
// Vector3DI::~Vector3DI() {}
// Vector3DI::Vector3DI (const VTYPE xa, const  VTYPE ya, const VTYPE za) {x=xa; y=ya; z=za;}
// Vector3DI::Vector3DI (const Vector2DC &op) {x=(VTYPE) op.x; y=(VTYPE) op.y; z=(VTYPE) 0;}
// Vector3DI::Vector3DI (const Vector2DI &op) {x=(VTYPE) op.x; y=(VTYPE) op.y; z=(VTYPE) 0;}
// Vector3DI::Vector3DI (const Vector2DF &op) {x=(VTYPE) op.x; y=(VTYPE) op.y; z=(VTYPE) 0;}
// Vector3DI::Vector3DI (const Vector3DC &op) {x=(VTYPE) op.x; y=(VTYPE) op.y; z=(VTYPE) op.z;}
// Vector3DI::Vector3DI (const Vector3DI &op) {x=(VTYPE) op.x; y=(VTYPE) op.y; z=(VTYPE) op.z;}
// Vector3DI::Vector3DI (const Vector3DF &op) {x=(VTYPE) op.x; y=(VTYPE) op.y; z=(VTYPE) op.z;}
// Vector3DI::Vector3DI (const Vector4DF &op) {x=(VTYPE) op.x; y=(VTYPE) op.y; z=(VTYPE) op.z;}
//
//// Set Functions
// Vector3DI &Vector3DI::Set (const int xa, const int ya, const int za)
//{
//	x = xa; y = ya; z = za;
//	return *this;
//}
//
//// Member Functions
// Vector3DI &Vector3DI::operator= (const Vector2DC &op) {x=(VTYPE) op.x; y=(VTYPE) op.y; return *this;}
// Vector3DI &Vector3DI::operator= (const Vector2DI &op) {x=(VTYPE) op.x; y=(VTYPE) op.y; return *this;}
// Vector3DI &Vector3DI::operator= (const Vector2DF &op) {x=(VTYPE) op.x; y=(VTYPE) op.y; return *this;}
// Vector3DI &Vector3DI::operator= (const Vector3DC &op) {x=(VTYPE) op.x; y=(VTYPE) op.y; z=(VTYPE) op.z; return *this;}
// Vector3DI &Vector3DI::operator= (const Vector3DI &op) {x=(VTYPE) op.x; y=(VTYPE) op.y; z=(VTYPE) op.z; return *this;}
// Vector3DI &Vector3DI::operator= (const Vector3DF &op) {x=(VTYPE) op.x; y=(VTYPE) op.y; z=(VTYPE) op.z; return *this;}
// Vector3DI &Vector3DI::operator= (const Vector4DF &op) {x=(VTYPE) op.x; y=(VTYPE) op.y; z=(VTYPE) op.z; return *this;}	
//	
// Vector3DI &Vector3DI::operator+= (const Vector2DC &op) {x+=(VTYPE) op.x; y+=(VTYPE) op.y; return *this;}
// Vector3DI &Vector3DI::operator+= (const Vector2DI &op) {x+=(VTYPE) op.x; y+=(VTYPE) op.y; return *this;}
// Vector3DI &Vector3DI::operator+= (const Vector2DF &op) {x+=(VTYPE) op.x; y+=(VTYPE) op.y; return *this;}
// Vector3DI &Vector3DI::operator+= (const Vector3DC &op) {x+=(VTYPE) op.x; y+=(VTYPE) op.y; z+=(VTYPE) op.z; return *this;}
// Vector3DI &Vector3DI::operator+= (const Vector3DI &op) {x+=(VTYPE) op.x; y+=(VTYPE) op.y; z+=(VTYPE) op.z; return *this;}
// Vector3DI &Vector3DI::operator+= (const Vector3DF &op) {x+=(VTYPE) op.x; y+=(VTYPE) op.y; z+=(VTYPE) op.z; return *this;}
// Vector3DI &Vector3DI::operator+= (const Vector4DF &op) {x+=(VTYPE) op.x; y+=(VTYPE) op.y; z+=(VTYPE) op.z; return *this;}
//
// Vector3DI &Vector3DI::operator-= (const Vector2DC &op) {x-=(VTYPE) op.x; y-=(VTYPE) op.y; return *this;}
// Vector3DI &Vector3DI::operator-= (const Vector2DI &op) {x-=(VTYPE) op.x; y-=(VTYPE) op.y; return *this;}
// Vector3DI &Vector3DI::operator-= (const Vector2DF &op) {x-=(VTYPE) op.x; y-=(VTYPE) op.y; return *this;}
// Vector3DI &Vector3DI::operator-= (const Vector3DC &op) {x-=(VTYPE) op.x; y-=(VTYPE) op.y; z-=(VTYPE) op.z; return *this;}
// Vector3DI &Vector3DI::operator-= (const Vector3DI &op) {x-=(VTYPE) op.x; y-=(VTYPE) op.y; z-=(VTYPE) op.z; return *this;}
// Vector3DI &Vector3DI::operator-= (const Vector3DF &op) {x-=(VTYPE) op.x; y-=(VTYPE) op.y; z-=(VTYPE) op.z; return *this;}
// Vector3DI &Vector3DI::operator-= (const Vector4DF &op) {x-=(VTYPE) op.x; y-=(VTYPE) op.y; z-=(VTYPE) op.z; return *this;}
//	
// Vector3DI &Vector3DI::operator*= (const Vector2DC &op) {x*=(VTYPE) op.x; y*=(VTYPE) op.y; return *this;}
// Vector3DI &Vector3DI::operator*= (const Vector2DI &op) {x*=(VTYPE) op.x; y*=(VTYPE) op.y; return *this;}
// Vector3DI &Vector3DI::operator*= (const Vector2DF &op) {x*=(VTYPE) op.x; y*=(VTYPE) op.y; return *this;}
// Vector3DI &Vector3DI::operator*= (const Vector3DC &op) {x*=(VTYPE) op.x; y*=(VTYPE) op.y; z*=(VTYPE) op.z; return *this;}
// Vector3DI &Vector3DI::operator*= (const Vector3DI &op) {x*=(VTYPE) op.x; y*=(VTYPE) op.y; z*=(VTYPE) op.z; return *this;}
// Vector3DI &Vector3DI::operator*= (const Vector3DF &op) {x*=(VTYPE) op.x; y*=(VTYPE) op.y; z*=(VTYPE) op.z; return *this;}
// Vector3DI &Vector3DI::operator*= (const Vector4DF &op) {x*=(VTYPE) op.x; y*=(VTYPE) op.y; z*=(VTYPE) op.z; return *this;}
//
// Vector3DI &Vector3DI::operator/= (const Vector2DC &op) {x/=(VTYPE) op.x; y/=(VTYPE) op.y; return *this;}
// Vector3DI &Vector3DI::operator/= (const Vector2DI &op) {x/=(VTYPE) op.x; y/=(VTYPE) op.y; return *this;}
// Vector3DI &Vector3DI::operator/= (const Vector2DF &op) {x/=(VTYPE) op.x; y/=(VTYPE) op.y; return *this;}
// Vector3DI &Vector3DI::operator/= (const Vector3DC &op) {x/=(VTYPE) op.x; y/=(VTYPE) op.y; z/=(VTYPE) op.z; return *this;}
// Vector3DI &Vector3DI::operator/= (const Vector3DI &op) {x/=(VTYPE) op.x; y/=(VTYPE) op.y; z/=(VTYPE) op.z; return *this;}
// Vector3DI &Vector3DI::operator/= (const Vector3DF &op) {x/=(VTYPE) op.x; y/=(VTYPE) op.y; z/=(VTYPE) op.z; return *this;}
// Vector3DI &Vector3DI::operator/= (const Vector4DF &op) {x/=(VTYPE) op.x; y/=(VTYPE) op.y; z/=(VTYPE) op.z; return *this;}
//
// Vector3DI &Vector3DI::Cross (const Vector3DC &v) {double ax = x, ay = y, az = z; x = (VTYPE) (ay * (double) v.z - az * (double) v.y); y = (VTYPE) (-ax * (double) v.z + az * (double) v.x); z = (VTYPE) (ax * (double) v.y - ay * (double) v.x); return *this;}
// Vector3DI &Vector3DI::Cross (const Vector3DI &v) {double ax = x, ay = y, az = z; x = (VTYPE) (ay * (double) v.z - az * (double) v.y); y = (VTYPE) (-ax * (double) v.z + az * (double) v.x); z = (VTYPE) (ax * (double) v.y - ay * (double) v.x); return *this;}
// Vector3DI &Vector3DI::Cross (const Vector3DF &v) {double ax = x, ay = y, az = z; x = (VTYPE) (ay * (double) v.z - az * (double) v.y); y = (VTYPE) (-ax * (double) v.z + az * (double) v.x); z = (VTYPE) (ax * (double) v.y - ay * (double) v.x); return *this;}
//		
// double Vector3DI::Dot(const Vector3DC &v)			{double dot; dot = (double) x*v.x + (double) y*v.y + (double) z*v.z; return dot;}
// double Vector3DI::Dot(const Vector3DI &v)			{double dot; dot = (double) x*v.x + (double) y*v.y + (double) z*v.z; return dot;}
// double Vector3DI::Dot(const Vector3DF &v)			{double dot; dot = (double) x*v.x + (double) y*v.y + (double) z*v.z; return dot;}
//
// double Vector3DI::Dist (const Vector2DC &v)		{ double distsq = DistSq (v); if (distsq!=0) return sqrt(distsq); return 0.0;}
// double Vector3DI::Dist (const Vector2DI &v)		{ double distsq = DistSq (v); if (distsq!=0) return sqrt(distsq); return 0.0;}
// double Vector3DI::Dist (const Vector2DF &v)		{ double distsq = DistSq (v); if (distsq!=0) return sqrt(distsq); return 0.0;}
// double Vector3DI::Dist (const Vector3DC &v)		{ double distsq = DistSq (v); if (distsq!=0) return sqrt(distsq); return 0.0;}
// double Vector3DI::Dist (const Vector3DI &v)		{ double distsq = DistSq (v); if (distsq!=0) return sqrt(distsq); return 0.0;}
// double Vector3DI::Dist (const Vector3DF &v)		{ double distsq = DistSq (v); if (distsq!=0) return sqrt(distsq); return 0.0;}
// double Vector3DI::Dist (const Vector4DF &v)		{ double distsq = DistSq (v); if (distsq!=0) return sqrt(distsq); return 0.0;}
// double Vector3DI::DistSq (const Vector2DC &v)		{ double a,b,c; a = (double) x - (double) v.x; b = (double) y - (double) v.y; c = (double) z; return (a*a + b*b + c*c);}
// double Vector3DI::DistSq (const Vector2DI &v)		{ double a,b,c; a = (double) x - (double) v.x; b = (double) y - (double) v.y; c = (double) z; return (a*a + b*b + c*c);}
// double Vector3DI::DistSq (const Vector2DF &v)		{ double a,b,c; a = (double) x - (double) v.x; b = (double) y - (double) v.y; c = (double) z; return (a*a + b*b + c*c);}
// double Vector3DI::DistSq (const Vector3DC &v)		{ double a,b,c; a = (double) x - (double) v.x; b = (double) y - (double) v.y; c = (double) z - (double) v.z; return (a*a + b*b + c*c);}
// double Vector3DI::DistSq (const Vector3DI &v)		{ double a,b,c; a = (double) x - (double) v.x; b = (double) y - (double) v.y; c = (double) z - (double) v.z; return (a*a + b*b + c*c);}
// double Vector3DI::DistSq (const Vector3DF &v)		{ double a,b,c; a = (double) x - (double) v.x; b = (double) y - (double) v.y; c = (double) z - (double) v.z; return (a*a + b*b + c*c);}
// double Vector3DI::DistSq (const Vector4DF &v)		{ double a,b,c; a = (double) x - (double) v.x; b = (double) y - (double) v.y; c = (double) z - (double) v.z; return (a*a + b*b + c*c);}
//
// Vector3DI &Vector3DI::Normalize (void) {
//	double n = (double) x*x + (double) y*y + (double) z*z;
//	if (n!=0.0) {
//		n = sqrt(n);
//		x = (VTYPE) (((double) x*255)/n);
//		y = (VTYPE) (((double) y*255)/n);
//		z = (VTYPE) (((double) z*255)/n);
//	}
//	return *this;
//}
// double Vector3DI::Length (void) { double n; n = (double) x*x + (double) y*y + (double) z*z; if (n != 0.0) return sqrt(n); return 0.0; }
// VTYPE *Vector3DI::Data (void)			{return &x;}
//
//#undef VTYPE
//#undef VNAME
//
//// Vector3DF Code Definition
//
//#define VNAME		3DF
//#define VTYPE		float
//
//// Constructors/Destructors
// Vector3DF::Vector3DF() {x=0; y=0; z=0;}
// Vector3DF::~Vector3DF() {}
// Vector3DF::Vector3DF (const VTYPE xa, const VTYPE ya, const VTYPE za) {x=xa; y=ya; z=za;}
// Vector3DF::Vector3DF (const Vector2DC &op) {x=(VTYPE) op.x; y=(VTYPE) op.y; z=(VTYPE) 0;}
// Vector3DF::Vector3DF (const Vector2DI &op) {x=(VTYPE) op.x; y=(VTYPE) op.y; z=(VTYPE) 0;}
// Vector3DF::Vector3DF (const Vector2DF &op) {x=(VTYPE) op.x; y=(VTYPE) op.y; z=(VTYPE) 0;}
// Vector3DF::Vector3DF (const Vector3DC &op) {x=(VTYPE) op.x; y=(VTYPE) op.y; z=(VTYPE) op.z;}
// Vector3DF::Vector3DF (const Vector3DI &op) {x=(VTYPE) op.x; y=(VTYPE) op.y; z=(VTYPE) op.z;}
// Vector3DF::Vector3DF (const Vector3DF &op) {x=(VTYPE) op.x; y=(VTYPE) op.y; z=(VTYPE) op.z;}
// Vector3DF::Vector3DF (const Vector4DF &op) {x=(VTYPE) op.x; y=(VTYPE) op.y; z=(VTYPE) op.z;}
//
//// Set Functions
// Vector3DF &Vector3DF::Set (const double xa, const double ya, const double za)
//{
//	x = (VTYPE) xa; y = (VTYPE) ya; z = (VTYPE) za;
//	return *this;
//}
//
//// Member Functions
// Vector3DF &Vector3DF::operator= (const int op) {x= (VTYPE) op; y= (VTYPE) op; z= (VTYPE) op; return *this;}
// Vector3DF &Vector3DF::operator= (const double op) {x= (VTYPE) op; y= (VTYPE) op; z= (VTYPE) op; return *this;}
// Vector3DF &Vector3DF::operator= (const Vector2DC &op) {x=(VTYPE) op.x; y=(VTYPE) op.y; return *this;}
// Vector3DF &Vector3DF::operator= (const Vector2DI &op) {x=(VTYPE) op.x; y=(VTYPE) op.y; return *this;}
// Vector3DF &Vector3DF::operator= (const Vector2DF &op) {x=(VTYPE) op.x; y=(VTYPE) op.y; return *this;}
// Vector3DF &Vector3DF::operator= (const Vector3DC &op) {x=(VTYPE) op.x; y=(VTYPE) op.y; z=(VTYPE) op.z; return *this;}
// Vector3DF &Vector3DF::operator= (const Vector3DI &op) {x=(VTYPE) op.x; y=(VTYPE) op.y; z=(VTYPE) op.z; return *this;}
// Vector3DF &Vector3DF::operator= (const Vector3DF &op) {x=(VTYPE) op.x; y=(VTYPE) op.y; z=(VTYPE) op.z; return *this;}
// Vector3DF &Vector3DF::operator= (const Vector4DF &op) {x=(VTYPE) op.x; y=(VTYPE) op.y; z=(VTYPE) op.z; return *this;}	
//	
// Vector3DF &Vector3DF::operator+= (const int op) {x+= (VTYPE) op; y+= (VTYPE) op; z+= (VTYPE) op; return *this;}
// Vector3DF &Vector3DF::operator+= (const double op) {x+= (VTYPE) op; y+= (VTYPE) op; z+= (VTYPE) op; return *this;}
// Vector3DF &Vector3DF::operator+= (const Vector2DC &op) {x+=(VTYPE) op.x; y+=(VTYPE) op.y; return *this;}
// Vector3DF &Vector3DF::operator+= (const Vector2DI &op) {x+=(VTYPE) op.x; y+=(VTYPE) op.y; return *this;}
// Vector3DF &Vector3DF::operator+= (const Vector2DF &op) {x+=(VTYPE) op.x; y+=(VTYPE) op.y; return *this;}
// Vector3DF &Vector3DF::operator+= (const Vector3DC &op) {x+=(VTYPE) op.x; y+=(VTYPE) op.y; z+=(VTYPE) op.z; return *this;}
// Vector3DF &Vector3DF::operator+= (const Vector3DI &op) {x+=(VTYPE) op.x; y+=(VTYPE) op.y; z+=(VTYPE) op.z; return *this;}
// Vector3DF &Vector3DF::operator+= (const Vector3DF &op) {x+=(VTYPE) op.x; y+=(VTYPE) op.y; z+=(VTYPE) op.z; return *this;}
// Vector3DF &Vector3DF::operator+= (const Vector4DF &op) {x+=(VTYPE) op.x; y+=(VTYPE) op.y; z+=(VTYPE) op.z; return *this;}
//
// Vector3DF &Vector3DF::operator-= (const int op) {x-= (VTYPE) op; y-= (VTYPE) op; z-= (VTYPE) op; return *this;}
// Vector3DF &Vector3DF::operator-= (const double op) {x-= (VTYPE) op; y-= (VTYPE) op; z-= (VTYPE) op; return *this;}
// Vector3DF &Vector3DF::operator-= (const Vector2DC &op) {x-=(VTYPE) op.x; y-=(VTYPE) op.y; return *this;}
// Vector3DF &Vector3DF::operator-= (const Vector2DI &op) {x-=(VTYPE) op.x; y-=(VTYPE) op.y; return *this;}
// Vector3DF &Vector3DF::operator-= (const Vector2DF &op) {x-=(VTYPE) op.x; y-=(VTYPE) op.y; return *this;}
// Vector3DF &Vector3DF::operator-= (const Vector3DC &op) {x-=(VTYPE) op.x; y-=(VTYPE) op.y; z-=(VTYPE) op.z; return *this;}
// Vector3DF &Vector3DF::operator-= (const Vector3DI &op) {x-=(VTYPE) op.x; y-=(VTYPE) op.y; z-=(VTYPE) op.z; return *this;}
// Vector3DF &Vector3DF::operator-= (const Vector3DF &op) {x-=(VTYPE) op.x; y-=(VTYPE) op.y; z-=(VTYPE) op.z; return *this;}
// Vector3DF &Vector3DF::operator-= (const Vector4DF &op) {x-=(VTYPE) op.x; y-=(VTYPE) op.y; z-=(VTYPE) op.z; return *this;}
//	
// Vector3DF &Vector3DF::operator*= (const int op) {x*= (VTYPE) op; y*= (VTYPE) op; z*= (VTYPE) op; return *this;}
// Vector3DF &Vector3DF::operator*= (const double op) {x*= (VTYPE) op; y*= (VTYPE) op; z*= (VTYPE) op; return *this;}
// Vector3DF &Vector3DF::operator*= (const Vector2DC &op) {x*=(VTYPE) op.x; y*=(VTYPE) op.y; return *this;}
// Vector3DF &Vector3DF::operator*= (const Vector2DI &op) {x*=(VTYPE) op.x; y*=(VTYPE) op.y; return *this;}
// Vector3DF &Vector3DF::operator*= (const Vector2DF &op) {x*=(VTYPE) op.x; y*=(VTYPE) op.y; return *this;}
// Vector3DF &Vector3DF::operator*= (const Vector3DC &op) {x*=(VTYPE) op.x; y*=(VTYPE) op.y; z*=(VTYPE) op.z; return *this;}
// Vector3DF &Vector3DF::operator*= (const Vector3DI &op) {x*=(VTYPE) op.x; y*=(VTYPE) op.y; z*=(VTYPE) op.z; return *this;}
// Vector3DF &Vector3DF::operator*= (const Vector3DF &op) {x*=(VTYPE) op.x; y*=(VTYPE) op.y; z*=(VTYPE) op.z; return *this;}
// Vector3DF &Vector3DF::operator*= (const Vector4DF &op) {x*=(VTYPE) op.x; y*=(VTYPE) op.y; z*=(VTYPE) op.z; return *this;}
//
// Vector3DF &Vector3DF::operator/= (const int op) {x/= (VTYPE) op; y/= (VTYPE) op; z/= (VTYPE) op; return *this;}
// Vector3DF &Vector3DF::operator/= (const double op) {x/= (VTYPE) op; y/= (VTYPE) op; z/= (VTYPE) op; return *this;}
// Vector3DF &Vector3DF::operator/= (const Vector2DC &op) {x/=(VTYPE) op.x; y/=(VTYPE) op.y; return *this;}
// Vector3DF &Vector3DF::operator/= (const Vector2DI &op) {x/=(VTYPE) op.x; y/=(VTYPE) op.y; return *this;}
// Vector3DF &Vector3DF::operator/= (const Vector2DF &op) {x/=(VTYPE) op.x; y/=(VTYPE) op.y; return *this;}
// Vector3DF &Vector3DF::operator/= (const Vector3DC &op) {x/=(VTYPE) op.x; y/=(VTYPE) op.y; z/=(VTYPE) op.z; return *this;}
// Vector3DF &Vector3DF::operator/= (const Vector3DI &op) {x/=(VTYPE) op.x; y/=(VTYPE) op.y; z/=(VTYPE) op.z; return *this;}
// Vector3DF &Vector3DF::operator/= (const Vector3DF &op) {x/=(VTYPE) op.x; y/=(VTYPE) op.y; z/=(VTYPE) op.z; return *this;}
// Vector3DF &Vector3DF::operator/= (const Vector4DF &op) {x/=(VTYPE) op.x; y/=(VTYPE) op.y; z/=(VTYPE) op.z; return *this;}
//
// Vector3DF &Vector3DF::Cross (const Vector3DC &v) {double ax = x, ay = y, az = z; x = (VTYPE) (ay * (double) v.z - az * (double) v.y); y = (VTYPE) (-ax * (double) v.z + az * (double) v.x); z = (VTYPE) (ax * (double) v.y - ay * (double) v.x); return *this;}
// Vector3DF &Vector3DF::Cross (const Vector3DI &v) {double ax = x, ay = y, az = z; x = (VTYPE) (ay * (double) v.z - az * (double) v.y); y = (VTYPE) (-ax * (double) v.z + az * (double) v.x); z = (VTYPE) (ax * (double) v.y - ay * (double) v.x); return *this;}
// Vector3DF &Vector3DF::Cross (const Vector3DF &v) {double ax = x, ay = y, az = z; x = (VTYPE) (ay * (double) v.z - az * (double) v.y); y = (VTYPE) (-ax * (double) v.z + az * (double) v.x); z = (VTYPE) (ax * (double) v.y - ay * (double) v.x); return *this;}
//		
// double Vector3DF::Dot(const Vector3DC &v)			{double dot; dot = (double) x*v.x + (double) y*v.y + (double) z*v.z; return dot;}
// double Vector3DF::Dot(const Vector3DI &v)			{double dot; dot = (double) x*v.x + (double) y*v.y + (double) z*v.z; return dot;}
// double Vector3DF::Dot(const Vector3DF &v)			{double dot; dot = (double) x*v.x + (double) y*v.y + (double) z*v.z; return dot;}
//
// double Vector3DF::Dist (const Vector2DC &v)		{ double distsq = DistSq (v); if (distsq!=0) return sqrt(distsq); return 0.0;}
// double Vector3DF::Dist (const Vector2DI &v)		{ double distsq = DistSq (v); if (distsq!=0) return sqrt(distsq); return 0.0;}
// double Vector3DF::Dist (const Vector2DF &v)		{ double distsq = DistSq (v); if (distsq!=0) return sqrt(distsq); return 0.0;}
// double Vector3DF::Dist (const Vector3DC &v)		{ double distsq = DistSq (v); if (distsq!=0) return sqrt(distsq); return 0.0;}
// double Vector3DF::Dist (const Vector3DI &v)		{ double distsq = DistSq (v); if (distsq!=0) return sqrt(distsq); return 0.0;}
// double Vector3DF::Dist (const Vector3DF &v)		{ double distsq = DistSq (v); if (distsq!=0) return sqrt(distsq); return 0.0;}
// double Vector3DF::Dist (const Vector4DF &v)		{ double distsq = DistSq (v); if (distsq!=0) return sqrt(distsq); return 0.0;}
// double Vector3DF::DistSq (const Vector2DC &v)		{ double a,b,c; a = (double) x - (double) v.x; b = (double) y - (double) v.y; c = (double) z; return (a*a + b*b + c*c);}
// double Vector3DF::DistSq (const Vector2DI &v)		{ double a,b,c; a = (double) x - (double) v.x; b = (double) y - (double) v.y; c = (double) z; return (a*a + b*b + c*c);}
// double Vector3DF::DistSq (const Vector2DF &v)		{ double a,b,c; a = (double) x - (double) v.x; b = (double) y - (double) v.y; c = (double) z; return (a*a + b*b + c*c);}
// double Vector3DF::DistSq (const Vector3DC &v)		{ double a,b,c; a = (double) x - (double) v.x; b = (double) y - (double) v.y; c = (double) z - (double) v.z; return (a*a + b*b + c*c);}
// double Vector3DF::DistSq (const Vector3DI &v)		{ double a,b,c; a = (double) x - (double) v.x; b = (double) y - (double) v.y; c = (double) z - (double) v.z; return (a*a + b*b + c*c);}
// double Vector3DF::DistSq (const Vector3DF &v)		{ double a,b,c; a = (double) x - (double) v.x; b = (double) y - (double) v.y; c = (double) z - (double) v.z; return (a*a + b*b + c*c);}
// double Vector3DF::DistSq (const Vector4DF &v)		{ double a,b,c; a = (double) x - (double) v.x; b = (double) y - (double) v.y; c = (double) z - (double) v.z; return (a*a + b*b + c*c);}
//
// Vector3DF &Vector3DF::Normalize (void) {
//	double n = (double) x*x + (double) y*y + (double)  z*z;
//	if (n!=0.0) {
//		n = sqrt(n);
//		x = double(x)/n; y = double(y)/n; z = double(z)/n;
//	}
//	return *this;
//}
// double Vector3DF::Length (void) { double n; n = (double) x*x + (double) y*y + (double) z*z; if (n != 0.0) return sqrt(n); return 0.0; }
// VTYPE *Vector3DF::Data ()			{return &x;}
//
//#undef VTYPE
//#undef VNAME
//
//// Vector4DC Code Definition
//
//#define VNAME		4DC
//#define VTYPE		unsigned char
//
//// Constructors/Destructors
// Vector4DC::Vector4DC() {x=0; y=0; z=0; w=0;}
// Vector4DC::~Vector4DC() {}
// Vector4DC::Vector4DC (const VTYPE xa, const VTYPE ya, const VTYPE za, const VTYPE wa) {x=xa; y=ya; z=za; w=wa;}
// Vector4DC::Vector4DC (const Vector2DC &op) {x=(VTYPE) op.x; y=(VTYPE) op.y; z=(VTYPE) 0; w=(VTYPE) 0;}
// Vector4DC::Vector4DC (const Vector2DI &op) {x=(VTYPE) op.x; y=(VTYPE) op.y; z=(VTYPE) 0; w=(VTYPE) 0;}
// Vector4DC::Vector4DC (const Vector2DF &op) {x=(VTYPE) op.x; y=(VTYPE) op.y; z=(VTYPE) 0; w=(VTYPE) 0;}
// Vector4DC::Vector4DC (const Vector3DC &op) {x=(VTYPE) op.x; y=(VTYPE) op.y; z=(VTYPE) op.z; w=(VTYPE) 0;}
// Vector4DC::Vector4DC (const Vector3DI &op) {x=(VTYPE) op.x; y=(VTYPE) op.y; z=(VTYPE) op.z; w=(VTYPE) 0;}
// Vector4DC::Vector4DC (const Vector3DF &op) {x=(VTYPE) op.x; y=(VTYPE) op.y; z=(VTYPE) op.z; w=(VTYPE) 0;}
// Vector4DC::Vector4DC (const Vector4DC &op) {x=(VTYPE) op.x; y=(VTYPE) op.y; z=(VTYPE) op.z; w=(VTYPE) op.w;}
// Vector4DC::Vector4DC (const Vector4DF &op) {x=(VTYPE) op.x; y=(VTYPE) op.y; z=(VTYPE) op.z; w=(VTYPE) op.w;}
//
//// Member Functions
// Vector4DC &Vector4DC::operator= (const int op) {x= (VTYPE) op; y= (VTYPE) op; z= (VTYPE) op; w = (VTYPE) op; return *this;}
// Vector4DC &Vector4DC::operator= (const double op) {x= (VTYPE) op; y= (VTYPE) op; z= (VTYPE) op; w = (VTYPE) op; return *this;}
// Vector4DC &Vector4DC::operator=  ( const Vector2DC &op) {x=(VTYPE) op.x; y=(VTYPE) op.y; z=(VTYPE) 0; w=(VTYPE) 0; return *this;}
// Vector4DC &Vector4DC::operator=  ( const Vector2DI &op) {x=(VTYPE) op.x; y=(VTYPE) op.y; z=(VTYPE) 0; w=(VTYPE) 0;  return *this;}
// Vector4DC &Vector4DC::operator=  ( const Vector2DF &op) {x=(VTYPE) op.x; y=(VTYPE) op.y; z=(VTYPE) 0; w=(VTYPE) 0;  return *this;}
// Vector4DC &Vector4DC::operator=  ( const Vector3DC &op) {x=(VTYPE) op.x; y=(VTYPE) op.y; z=(VTYPE) op.z; w=(VTYPE) 0;  return *this;}
// Vector4DC &Vector4DC::operator=  ( const Vector3DI &op) {x=(VTYPE) op.x; y=(VTYPE) op.y; z=(VTYPE) op.z; w=(VTYPE) 0;  return *this;}
// Vector4DC &Vector4DC::operator=  ( const Vector3DF &op) {x=(VTYPE) op.x; y=(VTYPE) op.y; z=(VTYPE) op.z; w=(VTYPE) 0; return *this;}
// Vector4DC &Vector4DC::operator=  ( const Vector4DC &op) {x=(VTYPE) op.x; y=(VTYPE) op.y; z=(VTYPE) op.z; w=(VTYPE) op.w; return *this;}	
// Vector4DC &Vector4DC::operator=  ( const Vector4DF &op) {x=(VTYPE) op.x; y=(VTYPE) op.y; z=(VTYPE) op.z; w=(VTYPE) op.w; return *this;}	
//	
// Vector4DC &Vector4DC::operator+= (const int op) {x+= (VTYPE) op; y+= (VTYPE) op; z+= (VTYPE) op; w += (VTYPE) op; return *this;}
// Vector4DC &Vector4DC::operator+= (const double op) {x+= (VTYPE) op; y+= (VTYPE) op; z+= (VTYPE) op; w += (VTYPE) op; return *this;}
// Vector4DC &Vector4DC::operator+=  ( const Vector2DC &op) {x+=(VTYPE) op.x; y+=(VTYPE) op.y; return *this;}
// Vector4DC &Vector4DC::operator+=  ( const Vector2DI &op) {x+=(VTYPE) op.x; y+=(VTYPE) op.y; return *this;}
// Vector4DC &Vector4DC::operator+=  ( const Vector2DF &op) {x+=(VTYPE) op.x; y+=(VTYPE) op.y; return *this;}
// Vector4DC &Vector4DC::operator+=  ( const Vector3DC &op) {x+=(VTYPE) op.x; y+=(VTYPE) op.y; z+=(VTYPE) op.z; return *this;}
// Vector4DC &Vector4DC::operator+=  ( const Vector3DI &op) {x+=(VTYPE) op.x; y+=(VTYPE) op.y; z+=(VTYPE) op.z; return *this;}
// Vector4DC &Vector4DC::operator+=  ( const Vector3DF &op) {x+=(VTYPE) op.x; y+=(VTYPE) op.y; z+=(VTYPE) op.z; return *this;}
// Vector4DC &Vector4DC::operator+=  ( const Vector4DC &op) {x+=(VTYPE) op.x; y+=(VTYPE) op.y; z+=(VTYPE) op.z; w+=(VTYPE) op.w; return *this;}	
// Vector4DC &Vector4DC::operator+=  ( const Vector4DF &op) {x+=(VTYPE) op.x; y+=(VTYPE) op.y; z+=(VTYPE) op.z; w+=(VTYPE) op.w; return *this;}	
//
// Vector4DC &Vector4DC::operator-= (const int op) {x-= (VTYPE) op; y-= (VTYPE) op; z-= (VTYPE) op; w -= (VTYPE) op; return *this;}
// Vector4DC &Vector4DC::operator-= (const double op) {x-= (VTYPE) op; y-= (VTYPE) op; z-= (VTYPE) op; w -= (VTYPE) op; return *this;}
// Vector4DC &Vector4DC::operator-=  ( const Vector2DC &op) {x-=(VTYPE) op.x; y-=(VTYPE) op.y; return *this;}
// Vector4DC &Vector4DC::operator-=  ( const Vector2DI &op) {x-=(VTYPE) op.x; y-=(VTYPE) op.y; return *this;}
// Vector4DC &Vector4DC::operator-=  ( const Vector2DF &op) {x-=(VTYPE) op.x; y-=(VTYPE) op.y; return *this;}
// Vector4DC &Vector4DC::operator-=  ( const Vector3DC &op) {x-=(VTYPE) op.x; y-=(VTYPE) op.y; z-=(VTYPE) op.z; return *this;}
// Vector4DC &Vector4DC::operator-=  ( const Vector3DI &op) {x-=(VTYPE) op.x; y-=(VTYPE) op.y; z-=(VTYPE) op.z; return *this;}
// Vector4DC &Vector4DC::operator-=  ( const Vector3DF &op) {x-=(VTYPE) op.x; y-=(VTYPE) op.y; z-=(VTYPE) op.z; return *this;}
// Vector4DC &Vector4DC::operator-=  ( const Vector4DC &op) {x-=(VTYPE) op.x; y-=(VTYPE) op.y; z-=(VTYPE) op.z; w-=(VTYPE) op.w; return *this;}	
// Vector4DC &Vector4DC::operator-=  ( const Vector4DF &op) {x-=(VTYPE) op.x; y-=(VTYPE) op.y; z-=(VTYPE) op.z; w-=(VTYPE) op.w; return *this;}	
//
// Vector4DC &Vector4DC::operator*= (const int op) {x*= (VTYPE) op; y*= (VTYPE) op; z*= (VTYPE) op; w *= (VTYPE) op; return *this;}
// Vector4DC &Vector4DC::operator*= (const double op) {x*= (VTYPE) op; y*= (VTYPE) op; z*= (VTYPE) op; w *= (VTYPE) op; return *this;}
// Vector4DC &Vector4DC::operator*=  ( const Vector2DC &op) {x*=(VTYPE) op.x; y*=(VTYPE) op.y; return *this;}
// Vector4DC &Vector4DC::operator*=  ( const Vector2DI &op) {x*=(VTYPE) op.x; y*=(VTYPE) op.y; return *this;}
// Vector4DC &Vector4DC::operator*=  ( const Vector2DF &op) {x*=(VTYPE) op.x; y*=(VTYPE) op.y; return *this;}
// Vector4DC &Vector4DC::operator*=  ( const Vector3DC &op) {x*=(VTYPE) op.x; y*=(VTYPE) op.y; z*=(VTYPE) op.z; return *this;}
// Vector4DC &Vector4DC::operator*=  ( const Vector3DI &op) {x*=(VTYPE) op.x; y*=(VTYPE) op.y; z*=(VTYPE) op.z; return *this;}
// Vector4DC &Vector4DC::operator*=  ( const Vector3DF &op) {x*=(VTYPE) op.x; y*=(VTYPE) op.y; z*=(VTYPE) op.z; return *this;}
// Vector4DC &Vector4DC::operator*=  ( const Vector4DC &op) {x*=(VTYPE) op.x; y*=(VTYPE) op.y; z*=(VTYPE) op.z; w*=(VTYPE) op.w; return *this;}	
// Vector4DC &Vector4DC::operator*=  ( const Vector4DF &op) {x*=(VTYPE) op.x; y*=(VTYPE) op.y; z*=(VTYPE) op.z; w*=(VTYPE) op.w; return *this;}	
//
// Vector4DC &Vector4DC::operator/= (const int op) {x/= (VTYPE) op; y/= (VTYPE) op; z/= (VTYPE) op; w /= (VTYPE) op; return *this;}
// Vector4DC &Vector4DC::operator/= (const double op) {x/= (VTYPE) op; y/= (VTYPE) op; z/= (VTYPE) op; w /= (VTYPE) op; return *this;}
// Vector4DC &Vector4DC::operator/=  ( const Vector2DC &op) {x/=(VTYPE) op.x; y/=(VTYPE) op.y; return *this;}
// Vector4DC &Vector4DC::operator/=  ( const Vector2DI &op) {x/=(VTYPE) op.x; y/=(VTYPE) op.y; return *this;}
// Vector4DC &Vector4DC::operator/=  ( const Vector2DF &op) {x/=(VTYPE) op.x; y/=(VTYPE) op.y; return *this;}
// Vector4DC &Vector4DC::operator/=  ( const Vector3DC &op) {x/=(VTYPE) op.x; y/=(VTYPE) op.y; z/=(VTYPE) op.z; return *this;}
// Vector4DC &Vector4DC::operator/=  ( const Vector3DI &op) {x/=(VTYPE) op.x; y/=(VTYPE) op.y; z/=(VTYPE) op.z; return *this;}
// Vector4DC &Vector4DC::operator/=  ( const Vector3DF &op) {x/=(VTYPE) op.x; y/=(VTYPE) op.y; z/=(VTYPE) op.z; return *this;}
// Vector4DC &Vector4DC::operator/=  ( const Vector4DC &op) {x/=(VTYPE) op.x; y/=(VTYPE) op.y; z/=(VTYPE) op.z; w/=(VTYPE) op.w; return *this;}	
// Vector4DC &Vector4DC::operator/=  ( const Vector4DF &op) {x/=(VTYPE) op.x; y/=(VTYPE) op.y; z/=(VTYPE) op.z; w/=(VTYPE) op.w; return *this;}
//
//	 Vector4DC Vector4DC::operator+ (const int op)			{ Vector4DC v(x,y,z,w); return v+=(VTYPE) op; }
//	 Vector4DC Vector4DC::operator+ (const float op)		{ Vector4DC v(x,y,z,w); return v+=(VTYPE) op; }
//	 Vector4DC Vector4DC::operator+  ( const Vector4DC &op)		{ Vector4DC v(x,y,z,w); return v+= op; }
//	 Vector4DC Vector4DC::operator- (const int op)			{ Vector4DC v(x,y,z,w); return v-=(VTYPE) op; }
//	 Vector4DC Vector4DC::operator- (const float op)		{ Vector4DC v(x,y,z,w); return v-=(VTYPE) op; }
//	 Vector4DC Vector4DC::operator-  ( const Vector4DC &op)		{ Vector4DC v(x,y,z,w); return v-= op; }
//	 Vector4DC Vector4DC::operator* (const int op)			{ Vector4DC v(x,y,z,w); return v*=(VTYPE) op; }
//	 Vector4DC Vector4DC::operator* (const float op)		{ Vector4DC v(x,y,z,w); return v*=(VTYPE) op; }
//	 Vector4DC Vector4DC::operator*  ( const Vector4DC &op)		{ Vector4DC v(x,y,z,w); return v*= op; }
//
// double Vector4DC::Dot ( const Vector4DF &v)			{double dot; dot = (double) x*v.x + (double) y*v.y + (double) z*v.z + (double) w*v.w; return dot;}
// double Vector4DC::Dist  ( const Vector4DF &v)		{double distsq = DistSq (v); if (distsq!=0) return sqrt(distsq); return 0.0;}
// double Vector4DC::DistSq  ( const Vector4DF &v)		{double a,b,c,d; a = (double) x - (double) v.x; b = (double) y - (double) v.y; c = (double) z - (double) v.z; d = (double) w - (double) v.w; return (a*a + b*b + c*c + d*d);}
//
// Vector4DC &Vector4DC::Normalize (void) {
//	float n = (float) x*x + (float) y*y + (float) z*z + (float) w*w;
//	if (n!=0.0) {
//		n = sqrt( n);
//		x = (VTYPE) (255.0 / n); y = (VTYPE) (255 / n); z = (VTYPE) (255 / n); w = (VTYPE) (255 / n);
//	}
//	return *this;
//}
// double Vector4DC::Length (void) { double n; n = (double) x*x + (double) y*y + (double) z*z + (double) w*w; if (n != 0.0) return sqrt(n); return 0.0; }
// VTYPE *Vector4DC::Data (void)			{return &x;}
//
//#undef VTYPE
//#undef VNAME
//
//
//// Vector4DF Code Definition
//
//#define VNAME		4DF
//#define VTYPE		float
//
//// Constructors/Destructors
// Vector4DF::Vector4DF() {x=0; y=0; z=0; w=0;}
// Vector4DF::~Vector4DF() {}
// Vector4DF::Vector4DF (const VTYPE xa, const VTYPE ya, const VTYPE za, const VTYPE wa) {x=xa; y=ya; z=za; w=wa;}
// Vector4DF::Vector4DF (const Vector2DC &op) {x=(VTYPE) op.x; y=(VTYPE) op.y; z=(VTYPE) 0; w=(VTYPE) 1;}
// Vector4DF::Vector4DF (const Vector2DI &op) {x=(VTYPE) op.x; y=(VTYPE) op.y; z=(VTYPE) 0; w=(VTYPE) 1;}
// Vector4DF::Vector4DF (const Vector2DF &op) {x=(VTYPE) op.x; y=(VTYPE) op.y; z=(VTYPE) 0; w=(VTYPE) 1;}
// Vector4DF::Vector4DF (const Vector3DC &op) {x=(VTYPE) op.x; y=(VTYPE) op.y; z=(VTYPE) op.z; w=(VTYPE) 1;}
// Vector4DF::Vector4DF (const Vector3DI &op) {x=(VTYPE) op.x; y=(VTYPE) op.y; z=(VTYPE) op.z; w=(VTYPE) 1;}
// Vector4DF::Vector4DF (const Vector3DF &op) {x=(VTYPE) op.x; y=(VTYPE) op.y; z=(VTYPE) op.z; w=(VTYPE) 1;}
// Vector4DF::Vector4DF (const Vector4DF &op) {x=(VTYPE) op.x; y=(VTYPE) op.y; z=(VTYPE) op.z; w=(VTYPE) op.w;}
//
//// Member Functions
// Vector4DF &Vector4DF::operator= (const int op) {x= (VTYPE) op; y= (VTYPE) op; z= (VTYPE) op; w = (VTYPE) op; return *this;}
// Vector4DF &Vector4DF::operator= (const double op) {x= (VTYPE) op; y= (VTYPE) op; z= (VTYPE) op; w = (VTYPE) op; return *this;}
// Vector4DF &Vector4DF::operator= (const Vector2DC &op) {x=(VTYPE) op.x; y=(VTYPE) op.y; z=(VTYPE) 0; w=(VTYPE) 0; return *this;}
// Vector4DF &Vector4DF::operator= (const Vector2DI &op) {x=(VTYPE) op.x; y=(VTYPE) op.y; z=(VTYPE) 0; w=(VTYPE) 0;  return *this;}
// Vector4DF &Vector4DF::operator= (const Vector2DF &op) {x=(VTYPE) op.x; y=(VTYPE) op.y; z=(VTYPE) 0; w=(VTYPE) 0;  return *this;}
// Vector4DF &Vector4DF::operator= (const Vector3DC &op) {x=(VTYPE) op.x; y=(VTYPE) op.y; z=(VTYPE) op.z; w=(VTYPE) 0;  return *this;}
// Vector4DF &Vector4DF::operator= (const Vector3DI &op) {x=(VTYPE) op.x; y=(VTYPE) op.y; z=(VTYPE) op.z; w=(VTYPE) 0;  return *this;}
// Vector4DF &Vector4DF::operator= (const Vector3DF &op) {x=(VTYPE) op.x; y=(VTYPE) op.y; z=(VTYPE) op.z; w=(VTYPE) 0; return *this;}
// Vector4DF &Vector4DF::operator= (const Vector4DF &op) {x=(VTYPE) op.x; y=(VTYPE) op.y; z=(VTYPE) op.z; w=(VTYPE) op.w; return *this;}	
//	
// Vector4DF &Vector4DF::operator+= (const int op) {x+= (VTYPE) op; y+= (VTYPE) op; z+= (VTYPE) op; w += (VTYPE) op; return *this;}
// Vector4DF &Vector4DF::operator+= (const float op) {x+= (VTYPE) op; y+= (VTYPE) op; z+= (VTYPE) op; w += (VTYPE) op; return *this;}
// Vector4DF &Vector4DF::operator+= (const double op) {x+= (VTYPE) op; y+= (VTYPE) op; z+= (VTYPE) op; w += (VTYPE) op; return *this;}
// Vector4DF &Vector4DF::operator+= (const Vector2DC &op) {x+=(VTYPE) op.x; y+=(VTYPE) op.y; return *this;}
// Vector4DF &Vector4DF::operator+= (const Vector2DI &op) {x+=(VTYPE) op.x; y+=(VTYPE) op.y; return *this;}
// Vector4DF &Vector4DF::operator+= (const Vector2DF &op) {x+=(VTYPE) op.x; y+=(VTYPE) op.y; return *this;}
// Vector4DF &Vector4DF::operator+= (const Vector3DC &op) {x+=(VTYPE) op.x; y+=(VTYPE) op.y; z+=(VTYPE) op.z; return *this;}
// Vector4DF &Vector4DF::operator+= (const Vector3DI &op) {x+=(VTYPE) op.x; y+=(VTYPE) op.y; z+=(VTYPE) op.z; return *this;}
// Vector4DF &Vector4DF::operator+= (const Vector3DF &op) {x+=(VTYPE) op.x; y+=(VTYPE) op.y; z+=(VTYPE) op.z; return *this;}
// Vector4DF &Vector4DF::operator+= (const Vector4DF &op) {x+=(VTYPE) op.x; y+=(VTYPE) op.y; z+=(VTYPE) op.z; w+=(VTYPE) op.w; return *this;}	
//
// Vector4DF &Vector4DF::operator-= (const int op) {x-= (VTYPE) op; y-= (VTYPE) op; z-= (VTYPE) op; w -= (VTYPE) op; return *this;}
// Vector4DF &Vector4DF::operator-= (const double op) {x-= (VTYPE) op; y-= (VTYPE) op; z-= (VTYPE) op; w -= (VTYPE) op; return *this;}
// Vector4DF &Vector4DF::operator-= (const Vector2DC &op) {x-=(VTYPE) op.x; y-=(VTYPE) op.y; return *this;}
// Vector4DF &Vector4DF::operator-= (const Vector2DI &op) {x-=(VTYPE) op.x; y-=(VTYPE) op.y; return *this;}
// Vector4DF &Vector4DF::operator-= (const Vector2DF &op) {x-=(VTYPE) op.x; y-=(VTYPE) op.y; return *this;}
// Vector4DF &Vector4DF::operator-= (const Vector3DC &op) {x-=(VTYPE) op.x; y-=(VTYPE) op.y; z-=(VTYPE) op.z; return *this;}
// Vector4DF &Vector4DF::operator-= (const Vector3DI &op) {x-=(VTYPE) op.x; y-=(VTYPE) op.y; z-=(VTYPE) op.z; return *this;}
// Vector4DF &Vector4DF::operator-= (const Vector3DF &op) {x-=(VTYPE) op.x; y-=(VTYPE) op.y; z-=(VTYPE) op.z; return *this;}
// Vector4DF &Vector4DF::operator-= (const Vector4DF &op) {x-=(VTYPE) op.x; y-=(VTYPE) op.y; z-=(VTYPE) op.z; w-=(VTYPE) op.w; return *this;}	
//
// Vector4DF &Vector4DF::operator*= (const int op) {x*= (VTYPE) op; y*= (VTYPE) op; z*= (VTYPE) op; w *= (VTYPE) op; return *this;}
// Vector4DF &Vector4DF::operator*= (const double op) {x*= (VTYPE) op; y*= (VTYPE) op; z*= (VTYPE) op; w *= (VTYPE) op; return *this;}
// Vector4DF &Vector4DF::operator*= (const Vector2DC &op) {x*=(VTYPE) op.x; y*=(VTYPE) op.y; return *this;}
// Vector4DF &Vector4DF::operator*= (const Vector2DI &op) {x*=(VTYPE) op.x; y*=(VTYPE) op.y; return *this;}
// Vector4DF &Vector4DF::operator*= (const Vector2DF &op) {x*=(VTYPE) op.x; y*=(VTYPE) op.y; return *this;}
// Vector4DF &Vector4DF::operator*= (const Vector3DC &op) {x*=(VTYPE) op.x; y*=(VTYPE) op.y; z*=(VTYPE) op.z; return *this;}
// Vector4DF &Vector4DF::operator*= (const Vector3DI &op) {x*=(VTYPE) op.x; y*=(VTYPE) op.y; z*=(VTYPE) op.z; return *this;}
// Vector4DF &Vector4DF::operator*= (const Vector3DF &op) {x*=(VTYPE) op.x; y*=(VTYPE) op.y; z*=(VTYPE) op.z; return *this;}
// Vector4DF &Vector4DF::operator*= (const Vector4DF &op) {x*=(VTYPE) op.x; y*=(VTYPE) op.y; z*=(VTYPE) op.z; w*=(VTYPE) op.w; return *this;}	
//
// Vector4DF &Vector4DF::operator/= (const int op) {x/= (VTYPE) op; y/= (VTYPE) op; z/= (VTYPE) op; w /= (VTYPE) op; return *this;}
// Vector4DF &Vector4DF::operator/= (const double op) {x/= (VTYPE) op; y/= (VTYPE) op; z/= (VTYPE) op; w /= (VTYPE) op; return *this;}
// Vector4DF &Vector4DF::operator/= (const Vector2DC &op) {x/=(VTYPE) op.x; y/=(VTYPE) op.y; return *this;}
// Vector4DF &Vector4DF::operator/= (const Vector2DI &op) {x/=(VTYPE) op.x; y/=(VTYPE) op.y; return *this;}
// Vector4DF &Vector4DF::operator/= (const Vector2DF &op) {x/=(VTYPE) op.x; y/=(VTYPE) op.y; return *this;}
// Vector4DF &Vector4DF::operator/= (const Vector3DC &op) {x/=(VTYPE) op.x; y/=(VTYPE) op.y; z/=(VTYPE) op.z; return *this;}
// Vector4DF &Vector4DF::operator/= (const Vector3DI &op) {x/=(VTYPE) op.x; y/=(VTYPE) op.y; z/=(VTYPE) op.z; return *this;}
// Vector4DF &Vector4DF::operator/= (const Vector3DF &op) {x/=(VTYPE) op.x; y/=(VTYPE) op.y; z/=(VTYPE) op.z; return *this;}
// Vector4DF &Vector4DF::operator/= (const Vector4DF &op) {x/=(VTYPE) op.x; y/=(VTYPE) op.y; z/=(VTYPE) op.z; w/=(VTYPE) op.w; return *this;}	
//
// Vector4DF &Vector4DF::Cross (const Vector4DF &v) {double ax = x, ay = y, az = z, aw = w; x = (VTYPE) (ay * (double) v.z - az * (double) v.y); y = (VTYPE) (-ax * (double) v.z + az * (double) v.x); z = (VTYPE) (ax * (double) v.y - ay * (double) v.x); w = (VTYPE) 0; return *this;}
//		
// double Vector4DF::Dot (const Vector4DF &v)			{double dot; dot = (double) x*v.x + (double) y*v.y + (double) z*v.z + (double) w*v.w; return dot;}
//
// double Vector4DF::Dist (const Vector4DF &v)		{double distsq = DistSq (v); if (distsq!=0) return sqrt(distsq); return 0.0;}
//
// double Vector4DF::DistSq (const Vector4DF &v)		{double a,b,c,d; a = (double) x - (double) v.x; b = (double) y - (double) v.y; c = (double) z - (double) v.z; d = (double) w - (double) v.w; return (a*a + b*b + c*c + d*d);}
//
// Vector4DF &Vector4DF::Normalize (void) {
//	VTYPE n = (VTYPE) x*x + (VTYPE) y*y + (VTYPE) z*z + (VTYPE) w*w;
//	if (n!=0.0) {
//		n = sqrt(n);
//		x /= n; y /= n; z /= n; w /= n;
//	}
//	return *this;
//}
// double Vector4DF::Length (void) { double n; n = (double) x*x + (double) y*y + (double) z*z + (double) w*w; if (n != 0.0) return sqrt(n); return 0.0; }
// VTYPE *Vector4DF::Data (void)			{return &x;}

#undef VTYPE
#undef VNAME

#define VNAME		2DC
#define VTYPE		unsigned char

// Vector2DC implementation
Vector2DC::Vector2DC(){x=0; y=0;};
Vector2DC::Vector2DC (VTYPE xa, VTYPE ya) {x=xa; y=ya;}
Vector2DC::Vector2DC (Vector2DC &op) {x=(VTYPE) op.x; y=(VTYPE) op.y;}
Vector2DC::Vector2DC (Vector2DI &op) {x=(VTYPE) op.x; y=(VTYPE) op.y;}

#undef VTYPE
#undef VNAME
