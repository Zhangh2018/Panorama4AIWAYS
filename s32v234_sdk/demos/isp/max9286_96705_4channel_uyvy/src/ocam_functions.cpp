/*------------------------------------------------------------------------------
   Example code that shows the use of the 'cam2world" and 'world2cam" functions
   Shows also how to undistort images into perspective or panoramic images
   Copyright (C) 2008 DAVIDE SCARAMUZZA, ETH Zurich
   Author: Davide Scaramuzza - email: davide.scaramuzza@ieee.org
------------------------------------------------------------------------------*/
#include "ocam_functions.h"
static const float PI = 3.1415926;
//------------------------------------------------------------------------------
int save_ocam_model_bin(const struct ocam_model *myocam_model,const char *filename)
{

}
int printf_ocam_model(const struct ocam_model *myocam_model,FILE *ofp)
{
       fprintf(ofp,"#polynomial coefficients for the DIRECT mapping function (ocam_model.ss in MATLAB). These are used by cam2world\n\n");
       fprintf(ofp,"%d ",myocam_model->length_pol);
       for(int i = 0;i < myocam_model->length_pol;i++)
       {
                fprintf(ofp,"%e ",myocam_model->pol[i]);
       }
       fprintf(ofp,"\n\n");
       fprintf(ofp,"#polynomial coefficients for the inverse mapping function (ocam_model.invpol in MATLAB). These are used by world2cam\n\n");
       fprintf(ofp,"%d ",myocam_model->length_invpol);
       for(int i = 0;i < myocam_model->length_invpol;i++)
       {
                fprintf(ofp,"%e ",myocam_model->invpol[i]);
       }
       fprintf(ofp,"\n\n");
       fprintf(ofp,"#center: \"row\" and \"column\", starting from 0 (C convention)\n\n");
       fprintf(ofp,"%f %f\n\n",myocam_model->xc,myocam_model->yc / myocam_model->width_rate);
       fprintf(ofp,"#affine parameters \"c\", \"d\", \"e\"\n\n");
       fprintf(ofp,"%f %f %f\n\n",myocam_model->c,myocam_model->d,myocam_model->e);
       fprintf(ofp,"#image size: \"height\" and \"width\"\n\n");
       fprintf(ofp,"%d %d\n\n",myocam_model->height,int(myocam_model->width / myocam_model->width_rate + 0.5));
}
int save_ocam_model(const struct ocam_model *myocam_model,const char *filename)
{
       FILE *ofp = fopen(filename,"w");
       if(ofp == NULL)
       {
                fprintf(stderr,"can not open the file: %s",filename);
                return -1;
       }
       fprintf(ofp,"#polynomial coefficients for the DIRECT mapping function (ocam_model.ss in MATLAB). These are used by cam2world\n\n");
       fprintf(ofp,"%d ",myocam_model->length_pol);
       for(int i = 0;i < myocam_model->length_pol;i++)
       {
                fprintf(ofp,"%e ",myocam_model->pol[i]);
       }
       fprintf(ofp,"\n\n");
       fprintf(ofp,"#polynomial coefficients for the inverse mapping function (ocam_model.invpol in MATLAB). These are used by world2cam\n\n");
       fprintf(ofp,"%d ",myocam_model->length_invpol);
       for(int i = 0;i < myocam_model->length_invpol;i++)
       {
                fprintf(ofp,"%e ",myocam_model->invpol[i]);
       }
       fprintf(ofp,"\n\n");
       fprintf(ofp,"#center: \"row\" and \"column\", starting from 0 (C convention)\n\n");
       fprintf(ofp,"%f %f\n\n",myocam_model->xc,myocam_model->yc / myocam_model->width_rate);
       fprintf(ofp,"#affine parameters \"c\", \"d\", \"e\"\n\n");
       fprintf(ofp,"%f %f %f\n\n",myocam_model->c,myocam_model->d,myocam_model->e);
       fprintf(ofp,"#image size: \"height\" and \"width\"\n\n");
       fprintf(ofp,"%d %d\n\n",myocam_model->height,int(myocam_model->width / myocam_model->width_rate + 0.5));
       fclose(ofp);
}
int get_ocam_model(struct ocam_model *myocam_model, const char *filename)
{
 double *pol        = myocam_model->pol;
 double *invpol     = myocam_model->invpol;
 double *xc         = &(myocam_model->xc);
 double *yc         = &(myocam_model->yc);
 double *c          = &(myocam_model->c);
 double *d          = &(myocam_model->d);
 double *e          = &(myocam_model->e);
 int    *width      = &(myocam_model->width);
 int    *height     = &(myocam_model->height);
 int    *length_pol    = &(myocam_model->length_pol);
 int    *length_invpol = &(myocam_model->length_invpol);
 FILE *f;
 char buf[CMV_MAX_BUF];
 int i;

 //Open file
 if(!(f=fopen(filename,"r")))
 {
   printf("File %s cannot be opened\n", filename);
   return -1;
 }

 //Read polynomial coefficients
 fgets(buf,CMV_MAX_BUF,f);
 fscanf(f,"\n");
 fscanf(f,"%d", length_pol);
 for (i = 0; i < *length_pol; i++)
 {
     fscanf(f," %lf",&pol[i]);
 }

 //Read inverse polynomial coefficients
 fscanf(f,"\n");
 fgets(buf,CMV_MAX_BUF,f);
 fscanf(f,"\n");
 fscanf(f,"%d", length_invpol);
 for (i = 0; i < *length_invpol; i++)
 {
     fscanf(f," %lf",&invpol[i]);
 }

 //Read center coordinates
 fscanf(f,"\n");
 fgets(buf,CMV_MAX_BUF,f);
 fscanf(f,"\n");
 fscanf(f,"%lf %lf\n", xc, yc);

 //Read affine coefficients
 fgets(buf,CMV_MAX_BUF,f);
 fscanf(f,"\n");
 fscanf(f,"%lf %lf %lf\n", c,d,e);

 //Read image size
 fgets(buf,CMV_MAX_BUF,f);
 fscanf(f,"\n");
 fscanf(f,"%d %d", height, width);

 fclose(f);
 return 0;
}

//------------------------------------------------------------------------------
void cam2world(double point3D[3], double point2D[2], struct ocam_model *myocam_model)
{
 double *pol    = myocam_model->pol;
 double xc      = (myocam_model->xc);
 double yc      = (myocam_model->yc);
 double c       = (myocam_model->c);
 double d       = (myocam_model->d);
 double e       = (myocam_model->e);
 int length_pol = (myocam_model->length_pol);
 double invdet  = 1/(c-d*e); // 1/det(A), where A = [c,d;e,1] as in the Matlab file

 double xp = invdet*(    (point2D[0] - xc) - d*(point2D[1] - yc) );
 double yp = invdet*( -e*(point2D[0] - xc) + c*(point2D[1] - yc) );

 double r   = sqrt(  xp*xp + yp*yp ); //distance [pixels] of  the point from the image center
 double zp  = pol[0];
 double r_i = 1;
 int i;

 for (i = 1; i < length_pol; i++)
 {
   r_i *= r;
   zp  += r_i*pol[i];
 }

 //normalize to unit norm
 double invnorm = 1/sqrt( xp*xp + yp*yp + zp*zp );

 point3D[0] = invnorm*xp;
 point3D[1] = invnorm*yp;
 point3D[2] = invnorm*zp;
}
//------------------------------------------------------------------------------

void world2cam(double point2D[2], double point3D[3], struct ocam_model *myocam_model)
{
 double *invpol     = myocam_model->invpol;
 double xc          = (myocam_model->xc);
 double yc          = (myocam_model->yc);
 double c           = (myocam_model->c);
 double d           = (myocam_model->d);
 double e           = (myocam_model->e);
 int    width       = (myocam_model->width);
 int    height      = (myocam_model->height);
 int length_invpol  = (myocam_model->length_invpol);
 double norm        = sqrt(point3D[0]*point3D[0] + point3D[1]*point3D[1]);
 double theta       = atan(point3D[2]/norm);
 double t, t_i;
 double rho, x, y;
 double invnorm;
 int i;

  if (norm != 0)
  {
    invnorm = 1/norm;
    t  = theta;
    rho = invpol[0];
    t_i = 1;

    for (i = 1; i < length_invpol; i++)
    {
      t_i *= t;
      rho += t_i*invpol[i];
    }

    x = point3D[0]*invnorm*rho;
    y = point3D[1]*invnorm*rho;


	point2D[0] = x*c + y*d + xc;
    point2D[1] = x*e + y*1 + yc;
  }
  else
  {
    point2D[0] = xc;
    point2D[1] = yc;
  }
}
/*
void cam2world(double point3D[3], double point2D[2], struct ocam_model *myocam_model,int srcWidth)
{
	double *pol    = myocam_model->pol;
	double xc      = (myocam_model->xc);
	double yc      = (myocam_model->yc);
	double c       = (myocam_model->c);
	double d       = (myocam_model->d);
	double e       = (myocam_model->e);
	int length_pol = (myocam_model->length_pol);
	double invdet  = 1/(c-d*e); // 1/det(A), where A = [c,d;e,1] as in the Matlab file

	double    width       = (myocam_model->width);
	double    height      = (myocam_model->height);


	double    lamd_y      = width/srcWidth;  //360为a当獭前°内ú参?标括定¨时骸图?像?宽í度è
	double    lamd_x      = lamd_y*(height*4/3)/(width);

	double xp = invdet*(    ((point2D[0] - xc)/lamd_x) - d*((point2D[1] - yc)/lamd_y) );
	double yp = invdet*( -e*((point2D[0] - xc)/lamd_x) + c*((point2D[1] - yc)/lamd_y) );
	//double xp = invdet*(    ((point2D[0] - xc)) - d*(point2D[1] - yc) );
	//double yp = invdet*( -e*((point2D[0] - xc)) + c*(point2D[1] - yc) );

	double r   = sqrt(  xp*xp + yp*yp ); //distance [pixels] of  the point from the image center
	double zp  = pol[0];
	double r_i = 1;
	int i;

	for (i = 1; i < length_pol; i++)
	{
		r_i *= r;
		zp  += r_i*pol[i];
	}

	//normalize to unit norm
	double invnorm = 1/sqrt( xp*xp + yp*yp + zp*zp );

	point3D[0] = invnorm*xp;
	point3D[1] = invnorm*yp;
	point3D[2] = invnorm*zp;
}

//------------------------------------------------------------------------------
void world2cam(double point2D[2], double point3D[3], struct ocam_model *myocam_model,int srcWidth)
{
	double *invpol     = myocam_model->invpol;
	double xc          = (myocam_model->xc);
	double yc          = (myocam_model->yc);
	double c           = (myocam_model->c);
	double d           = (myocam_model->d);
	double e           = (myocam_model->e);
	double    width       = (myocam_model->width);
	double    height      = (myocam_model->height);
	int length_invpol  = (myocam_model->length_invpol);
	double norm        = sqrt(point3D[0]*point3D[0] + point3D[1]*point3D[1]);
	double theta       = atan(point3D[2]/norm);
	double t, t_i;
	double rho, x, y;
	double invnorm;
	int i;

	double    lamd_y    =  width/srcWidth;
	double    lamd_x    =  lamd_y*((height*4/3)/(width));

	if (norm != 0)
	{
		invnorm = 1/norm;
		t  = theta;
		rho = invpol[0];
		t_i = 1;

		for (i = 1; i < length_invpol; i++)
		{
			t_i *= t;
			rho += t_i*invpol[i];
		}

		x = point3D[0]*invnorm*rho*lamd_x;
		y = point3D[1]*invnorm*rho*lamd_y;
		// 	x = point3D[0]*invnorm*rho;
		//y = point3D[1]*invnorm*rho;

		point2D[0] = x*c + y*d + xc;
		point2D[1] = x*e + y*1 + yc;
	}
	else
	{
		point2D[0] = xc;
		point2D[1] = yc;
	}
}
*/
void cam2world(double point3D[3], double point2D[2], struct ocam_model *myocam_model,int srcWidth)
{
	double *pol    = myocam_model->pol;
	double xc      = (myocam_model->xc);
	double yc      = (myocam_model->yc);
	double c       = (myocam_model->c);
	double d       = (myocam_model->d);
	double e       = (myocam_model->e);
	int length_pol = (myocam_model->length_pol);
	double invdet  = 1/(c-d*e); // 1/det(A), where A = [c,d;e,1] as in the Matlab file
//    printf("xc,yc,c,d,e,length_pol,invdet = (%f,%f,%f,%f,%f,%d,%f)\n",xc,yc,c,d,e,length_pol,invdet);
	double    width       = (myocam_model->width);
	double    height      = (myocam_model->height);


	double    lamd_y      = myocam_model->width_rate;
	double    lamd_x      = lamd_y*(height*4/3)/(width);


	double xp = invdet*(    ((point2D[0] - xc)/lamd_x) - d*((point2D[1] - yc)/lamd_y) );
	double yp = invdet*( -e*((point2D[0] - xc)/lamd_x) + c*((point2D[1] - yc)/lamd_y) );

	double r   = sqrt(  xp*xp + yp*yp ); //distance [pixels] of  the point from the image center

//	printf("width,height,lamd_y,lamd_x,xp,yp,r = (%f,%f,%f,%f,%f,%f,%f)\n",width,height,lamd_y,lamd_x,xp,yp,r);
	double zp  = pol[0];
	double r_i = 1;
	int i;

	for (i = 1; i < length_pol; i++)
	{
		r_i *= r;
		zp  += r_i*pol[i];
	}

	//normalize to unit norm
	double invnorm = 1/sqrt( xp*xp + yp*yp + zp*zp );

	point3D[0] = invnorm*xp;
	point3D[1] = invnorm*yp;
	point3D[2] = invnorm*zp;

//	printf("invnorm,point3D[0],point3D[1],point3D[2] = (%f,%f,%f,%f)\n",invnorm,point3D[0],point3D[1],point3D[2]);
}
//------------------------------------------------------------------------------
void world2cam(double point2D[2], double point3D[3], struct ocam_model *myocam_model,int srcWidth)
{
	double *invpol     = myocam_model->invpol;
	double xc          = (myocam_model->xc);
	double yc          = (myocam_model->yc);
	double c           = (myocam_model->c);
	double d           = (myocam_model->d);
	double e           = (myocam_model->e);
	double    width       = (myocam_model->width);
	double    height      = (myocam_model->height);
	int length_invpol  = (myocam_model->length_invpol);
	double norm        = sqrt(point3D[0]*point3D[0] + point3D[1]*point3D[1]);
	double theta       = atan(point3D[2]/norm);
	double t, t_i;
	double rho, x, y;
	double invnorm;
	int i;

	double    lamd_y      = myocam_model->width_rate;
	double    lamd_x      = lamd_y*(height*4/3)/(width);

	if (norm != 0)
	{
		invnorm = 1/norm;
		t  = theta;
		rho = invpol[0];
		t_i = 1;

		for (i = 1; i < length_invpol; i++)
		{
			t_i *= t;
			rho += t_i*invpol[i];
		}

		x = point3D[0]*invnorm*rho*lamd_x;
		y = point3D[1]*invnorm*rho*lamd_y;

		point2D[0] = x*c + y*d + xc;
		point2D[1] = x*e + y*1 + yc;
	}
	else
	{
		point2D[0] = xc;
		point2D[1] = yc;
	}
}
int get_ocam_model(struct ocam_model *myocam_model, const char *filename,int srcWidth,int srcHeight)
{
	double *pol        = myocam_model->pol;
	double *invpol     = myocam_model->invpol;
	double *xc         = &(myocam_model->xc);
	double *yc         = &(myocam_model->yc);
	double *c          = &(myocam_model->c);
	double *d          = &(myocam_model->d);
	double *e          = &(myocam_model->e);
	int    *width      = &(myocam_model->width);
	int    *height     = &(myocam_model->height);
	int *length_pol    = &(myocam_model->length_pol);
	int *length_invpol = &(myocam_model->length_invpol);
	float *width_rate  = &(myocam_model->width_rate);
	FILE *f;
	char buf[CMV_MAX_BUF];
	int i;

	//Open file
	if(!(f=fopen(filename,"r")))
	{
		printf("File %s cannot be opened\n", filename);
		return -1;
	}

	//Read polynomial coefficients
	fgets(buf,CMV_MAX_BUF,f);
	fscanf(f,"\n");
	fscanf(f,"%d", length_pol);
	for (i = 0; i < *length_pol; i++)
	{
		fscanf(f," %lf",&pol[i]);
	}

	//Read inverse polynomial coefficients
	fscanf(f,"\n");
	fgets(buf,CMV_MAX_BUF,f);
	fscanf(f,"\n");
	fscanf(f,"%d", length_invpol);
	for (i = 0; i < *length_invpol; i++)
	{
		fscanf(f," %lf",&invpol[i]);
	}

	//Read center coordinates
	fscanf(f,"\n");
	fgets(buf,CMV_MAX_BUF,f);
	fscanf(f,"\n");
	fscanf(f,"%lf %lf\n", xc, yc);

	//Read affine coefficients
	fgets(buf,CMV_MAX_BUF,f);
	fscanf(f,"\n");
	fscanf(f,"%lf %lf %lf\n", c,d,e);

	//Read image size
	fgets(buf,CMV_MAX_BUF,f);
	fscanf(f,"\n");
	fscanf(f,"%d %d", height, width);

	*width_rate = ((double)srcWidth)/(*width);
	*xc =(*xc) * ((double)srcHeight)/(*height);
	*yc =(*yc) * ((double)srcWidth)/(*width);

	*height = srcHeight;
	*width  = srcWidth;

	fclose(f);
	return 0;
}
