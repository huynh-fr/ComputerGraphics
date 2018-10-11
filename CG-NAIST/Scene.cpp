// Scene.cpp: implementation.

#include "Scene.h"
#include <stdio.h>
#include <iostream>
#include <iomanip>

namespace ComputerGraphicsCourse
{
	Image* RayTrace(const Scene& scene, const Camera& camera)
	{
		const int width = camera.Width, height = camera.Height;
		Image* image = new Image(width, height);
		int count = 0;

		for (int j = 0; j < height; j++)
		{
#ifdef _OPENMP
#pragma omp parallel for
#endif
			for (int i = 0; i < width; i++)
			{
				// YOUR CODE FOR ASSIGNMENT 0 HERE.
				
				image->Data[width*j + i] = Eigen::Vector3d(1, 0.5*i/width, 0.1); /* Background Color */

				Ray ray = RayThruPixel(camera, i, j);
				IntersectionInfo hit = Intersect(ray, scene);
				if (hit.HitObject != NULL) {
					image->Data[width*j + i] = FindColor(scene, hit, scene.MaxDepth);
				}
			}
			std::cout << "\rRender: " << std::setprecision(2) << std::fixed << (100.0*(count++)) / (height - 1) << "%" << std::flush;
		}
		return image;
	}

	Ray RayThruPixel(const Camera& camera, const int i, const int j)
	{
		// YOUR CODE FOR ASSIGNMENT 2 HERE.  
		Eigen::Vector3d origin = Eigen::Vector3d((i - camera.Width / 2) / 100., (camera.Height / 2 - j) / 100., 1);
		Eigen::Vector3d direction = Eigen::Vector3d(0, 0, -1);
		return Ray(origin, direction);
	}

	IntersectionInfo Intersect(const Ray& ray, const Scene& scene)
	{
		double mindist = 1e8;
		Object* hitobject = NULL;
		Eigen::Vector3d hitpos(0,0,0);	// hit position
		Eigen::Vector3d normal(0, 0, 0);	// hit position normal
		for (std::vector<Object*>::const_iterator o = scene.objList.begin(); o < scene.objList.end(); o++) // find closest intersection; test all objects
		{
			Eigen::Vector3d p, n;
			double t = Intersect(ray, *o, p, n);
			if (t > 0 && t < mindist) // closer than previous closest object
			{
				mindist = t;
				hitobject = *o;
				hitpos = p;
				normal = n;
			}
		}

		return IntersectionInfo(hitobject, mindist, hitpos, normal, ray);	// may already be in Intersect()
	}


	double Intersect(const Ray& ray, const Object* obj, Eigen::Vector3d &position, Eigen::Vector3d &normal)
	{
		Eigen::Vector4d p0(ray.P0[0], ray.P0[1], ray.P0[2], 1), p1(ray.P1[0], ray.P1[1], ray.P1[2], 0);
		// invert transform
		// YOUR CODE FOR ASSIGNMENT 4 HERE.  

		Ray transformedRay(p0.block<3, 1>(0, 0), p1.block<3, 1>(0, 0));
		double t = obj->Intersect(transformedRay, position, normal);

		if (t < 0) return t;

		// transform the results
		// YOUR CODE FOR ASSIGNMENT 4 HERE.  

		return t;
	}


	double Triangle::Intersect(const Ray& ray, Eigen::Vector3d &position, Eigen::Vector3d &normal) const
	{
		// YOUR CODE FOR ASSIGNMENT 1 HERE.  
		/*TODO: Implement ray-triangle intersection. */
		
		/****************************/
		/***Ray Plane Intersection***/
		/****************************/
		
		double t, v, w;
		
		/*vertice0,1,2 are vectors a,b,c*/
		
		t = ((*vertices[0]).dot(n) - ray.P0.dot(n)) / (ray.P1.dot(n)); //t distance between viewpt and intersection pt
		Eigen::Vector3d P = ray.P0 + ray.P1*t; //position of point in triangle's plan

		Eigen::Vector3d uvw = inv*(P - (*vertices[0]));

		v = uvw[1];
		w = uvw[2];

		/*Storage of position and normal*/
		position = P;
		normal = n;

		if (v >= 0 && w>= 0 && v + w <= 1) //intersection of ray with the triangle
			return t;
		else
			return -1;
	}


	double Sphere::Intersect(const Ray& ray, Eigen::Vector3d &position, Eigen::Vector3d &normal) const
	{
		// YOUR CODE FOR ASSIGNMENT 2 HERE.  
		/*TODO: Implement ray-sphere intersection. */
		/* return positive distance from ray origin to intersection */
		/* return -1, if no sphere intersects */

		/* solve the eq. : at^2 + bt + c = 0 */


		/* Complex roots: no intersection */

		/* 2 roots found */

		/* 2 real positive roots: pick smaller root */

		/* One positive, one negative root: ray origin inside sphere (pick + root) */

		/* Both negative, no intersection */
		return -1;
	}

#ifdef TRIANGLE_NORMAL
	double TriangleNormal::Intersect(const Ray& ray, Eigen::Vector3d &position, Eigen::Vector3d &normal) const
	{
		/*TODO: Implement ray-triangle intersection. */
		// 1. Ray-triangle intersection

		// 2. interpolate normal by coefficient


		return -1;
	}
#endif

	Eigen::Vector3d FindColor(const Scene& scene, const IntersectionInfo& hit, const int depth)
	{
		if (hit.HitObject == NULL) return Eigen::Vector3d(0, 0, 0);
		Material mtrl(hit.HitObject->material);
		Eigen::Vector3d color = mtrl.ambient + mtrl.emission;

		/* Ignore Ka, Ke, and Ks terms from the shading equation on pp.7 */
		Eigen::Vector3d p = hit.pos;
		Eigen::Vector3d n = hit.nor.normalized();

		// YOUR CODE FOR ASSIGNMENT 3 HERE.  

		// Execute the following processes for each light
		// 1. calculate direction and intensity of incoming light

		// 2. check if the light is visible from the hit point or not

		// 3. calculate diffuse and specular shading 

		// 4. *option* calculate recursive specular reflection

		return color;
	}


	// Helper rotation function.  Please implement this.  
	Eigen::Matrix4d Transform::rotate(const float degrees, const Eigen::Vector3d& axis)
	{
		Eigen::Matrix4d ret = Eigen::Matrix4d::Identity();
		// YOUR CODE FOR ASSIGNMENT 4 HERE.  

		return ret;
	}

	Eigen::Matrix4d Transform::scale(const float &sx, const float &sy, const float &sz)
	{
		Eigen::Matrix4d ret = Eigen::Matrix4d::Identity();
		// YOUR CODE FOR ASSIGNMENT 4 HERE.  

		return ret;
	}

	Eigen::Matrix4d Transform::translate(const float &tx, const float &ty, const float &tz)
	{
		Eigen::Matrix4d ret = Eigen::Matrix4d::Identity();
		// YOUR CODE FOR ASSIGNMENT 4 HERE.  

		return ret;
	}

	Eigen::Vector3d Light::Ambient = Eigen::Vector3d(0.2, 0.2, 0.2);
	Eigen::Vector3d Light::Attenuation = Eigen::Vector3d(1., 0., 0.);

	Eigen::Vector3d Material::Diffuse = Eigen::Vector3d(0., 0., 0.);
	Eigen::Vector3d Material::Emission = Eigen::Vector3d(0., 0., 0.);
	Eigen::Vector3d Material::Specular = Eigen::Vector3d(0., 0., 0.);
	float Material::Shininess = 0.f;

}