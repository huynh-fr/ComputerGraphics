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
		Eigen::Vector3d u, v, w, P0, P1;
		double alpha, beta;

		P0 = camera.LookFrom;
		w = camera.LookFrom - camera.LookAt;
		w.normalize();
		u = -w.cross(camera.Up).normalized();
		v = u.cross(-w);

		alpha = tan(camera.FoV_X / 2.)*(i - (camera.Width / 2.)) / (camera.Width / 2.);
		beta = tan(camera.FoV_Y / 2.)*((camera.Height / 2.) - j) / (camera.Height / 2.);

		P1 = (alpha*u + beta * v - w).normalized();

		Eigen::Vector3d origin = P0; //Eigen::Vector3d((i - camera.Width / 2) / 100., (camera.Height / 2 - j) / 100., 1);
		Eigen::Vector3d direction = P1; // Eigen::Vector3d(0, 0, -1);
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
		Eigen::Vector4d p0(ray.P0[0], ray.P0[1], ray.P0[2], 1), //1:position
			p1(ray.P1[0], ray.P1[1], ray.P1[2], 0); //0:vector
		
		// YOUR CODE FOR ASSIGNMENT 4 HERE.  
		// Inversely transform p0 and p1 here.
		// p0 = M-1*p0, p1=M-1*p1 (contravariant vector)
		Eigen::Matrix4d M = obj->transform;
		p0 = M.inverse()*p0;
		p1 = M.inverse()*p1;

		Ray transformedRay(p0.block<3, 1>(0, 0), p1.block<3, 1>(0, 0));
		double t = obj->Intersect(transformedRay, position, normal);

		if (t < 0) return t;

		// transform the results (position and normal)
		// YOUR CODE FOR ASSIGNMENT 4 HERE.  
		Eigen::Vector4d pos(position[0], position[1], position[2], 1), 
			norm(normal[0], normal[1], normal[2], 0);
		
		// pos = M*pos, norm = (M-1).transpose() * norm (covariant vector)
		// set pos and norm into position and normal, respectively.
		pos = M*pos;
		norm = M.inverse().transpose()*norm;

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

		double a, b, c, delta, t;

		a = ray.P1.dot(ray.P1);
		b = 2 * ray.P1.dot(ray.P0 - C);
		c = (ray.P0 - C).dot(ray.P0 - C) - pow(Radius,2);

		/* solve the eq. : at^2 + bt + c = 0 */
		delta = pow(b,2) - 4 * a*c;

		/* Complex roots: no intersection */
		if (delta < 0)
			return -1;
		
		else if (delta == 0) //intersects with sphere surface
			t = -b / (2 * a);
		
		else //intersects the sphere on 2 points
		{
			/* 2 roots found */
			double t1, t2;

			t1 = (-b - sqrt(delta)) / (2 * a);
			t2 = (-b + sqrt(delta)) / (2 * a);
			
			/* 2 real positive roots: pick smaller root */
			if (t1 > 0 && t2 > 0)
				t = fmin(t1, t2);

			/* One positive, one negative root: ray origin inside sphere (pick + root) */
			else if (t1 > 0 && t2 < 0)
				t = t1;

			else if (t1 < 0 && t2 > 0)
				t = t2;

			/* Both negative, no intersection */
			else
				return -1;
		}

		position = ray.P0 + ray.P1*t;
		normal = (position - C) / (position - C).norm();

		return t;
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
		for (unsigned int i = 0; i < scene.lights.size(); i++)
		{
			Light light = scene.lights[i];
			Eigen::Vector3d lightPos;
			lightPos[0] = light.position[0];
			lightPos[1] = light.position[1];
			lightPos[2]= light.position[2];
			Eigen::Vector3d intensity = light.color;
			Eigen::Vector3d dir;

			// 1. calculate direction and intensity of incoming light

			/*Point light*/
			if (light.position[3] == 1)
			{
				/*Direction = everywhere*/
				dir = lightPos - p;

				/*Intensity = with attenuation*/
				intensity /= light.attenuation[0] + light.attenuation[1] * dir.norm() + light.attenuation[2] * pow(dir.norm(),2);
			}
			/*Directional light*/
			else // == 0
			{
				/*Direction = light.position[n] are vector coordinates*/
				dir = - lightPos;
				/*Intensity = homogeneous, so we keep intensity variable as it is*/
			}
			

			// 2. check if the light is visible from the hit point or not
			Ray shadowRay(p+0.00001*dir, dir); //from hit point to light src
			IntersectionInfo sHit = Intersect(shadowRay, scene);

			if (sHit.HitObject != NULL) {
				// if the shadow ray hits an object, skip the light
				continue;
			}

			// 3. calculate diffuse and specular shading 
			Eigen::Vector3d L = shadowRay.P1.normalized();
			Eigen::Vector3d H = (dir.normalized() - hit.ray.P1).normalized();

			/*Diffuse color(RGB) * light color(RGB) * reflection term(scalar)*/
			color += mtrl.diffuse.cwiseProduct(intensity)*fmax(L.dot(n),0);
			
			/*specular color(RGB) * light color(RGB) * reflection term(scalar)*/
			
			color += mtrl.specular.cwiseProduct(intensity)*pow(fmax(H.dot(n), 0),mtrl.shininess);

			// 4. *option* calculate recursive specular reflection

		}
			
		return color;
	}


	// Helper rotation function.  Please implement this.  
	Eigen::Matrix4d Transform::rotate(const float degrees, const Eigen::Vector3d& axis)
	{
		Eigen::Matrix4d ret = Eigen::Matrix4d::Identity();
		Eigen::Matrix3d A = Eigen::Matrix3d::Zero();
		Eigen::Matrix3d R;
		// YOUR CODE FOR ASSIGNMENT 4 HERE. 
		A(0, 1) = -1 * axis[2];
		A(0, 2) = axis[1];
		A(1, 0) = axis[0];
		A(1, 2) = -1 * axis[0];
		A(2, 0) = -1 * axis[1];
		A(2, 1) = axis[0];

		R = cos(degrees)*Eigen::Matrix3d::Identity() + (1 - cos(degrees))*axis*axis.transpose() + sin(degrees)*A;

		/*Eigen::Vector3d axis_x(1, 0, 0);
		Eigen::Vector3d axis_y(0, 1, 0);
		Eigen::Vector3d axis_z(0, 0, 1);

		if (axis == axis_x)
		{
			ret(1, 1) = cos(degrees);
			ret(1, 2) = -sin(degrees);
			ret(2, 1) = sin(degrees);
			ret(2, 2) = cos(degrees);
		}
		else if (axis == axis_y)
		{
			ret(0, 0) = cos(degrees);
			ret(0, 2) = sin(degrees);
			ret(2, 0) = -sin(degrees);
			ret(2, 2) = cos(degrees);
		}
		else if (axis == axis_z)
		{
			ret(0, 0) = cos(degrees);
			ret(0, 1) = -sin(degrees);
			ret(1, 0) = sin(degrees);
			ret(1, 1) = cos(degrees);
		}*/

		for (int i = 0; i < 3; i++)
		{
			for (int j = 0; j < 3; j++)
				ret(i, j) = R(i, j);
		}

		return ret;
	}

	Eigen::Matrix4d Transform::scale(const float &sx, const float &sy, const float &sz)
	{
		Eigen::Matrix4d ret = Eigen::Matrix4d::Identity();
		// YOUR CODE FOR ASSIGNMENT 4 HERE.  
		ret(0,0) = sx;
		ret(1,1) = sy;
		ret(2,2) = sz;

		return ret;
	}

	Eigen::Matrix4d Transform::translate(const float &tx, const float &ty, const float &tz)
	{
		Eigen::Matrix4d ret = Eigen::Matrix4d::Identity();
		// YOUR CODE FOR ASSIGNMENT 4 HERE.  
		ret(0, 3) = tx;
		ret(1, 3) = ty;
		ret(2, 3) = tz;

		return ret;
	}

	Eigen::Vector3d Light::Ambient = Eigen::Vector3d(0.2, 0.2, 0.2);
	Eigen::Vector3d Light::Attenuation = Eigen::Vector3d(1., 0., 0.);

	Eigen::Vector3d Material::Diffuse = Eigen::Vector3d(0., 0., 0.);
	Eigen::Vector3d Material::Emission = Eigen::Vector3d(0., 0., 0.);
	Eigen::Vector3d Material::Specular = Eigen::Vector3d(0., 0., 0.);
	float Material::Shininess = 0.f;

}