using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Particle : MonoBehaviour
{
    //Properties
    private float r;        //Radius of particle
    public GameObject obj; //Associated object
    private Vector3 pos;    //Current position
    private Vector3 vel;    //Current velocity
    private float den;    //Density
    private float pres;    //Pressure
    private Vector3 force;  //Total force acting on particle


    //****Get and Set functions
    public void setRadius(float rad){
        r = rad;
        gameObject.transform.localScale = new Vector3(r, r, r);
    }

    public void setPosition(Vector3 p){
        pos = p;
        gameObject.transform.position = pos;
    }
    public Vector3 getPosition(){
        return pos;
    }
    
    public void setVelocity(Vector3 v){
        vel = v;
    }
    public Vector3 getVelocity(){
        return vel;
    }

    public void setDensity(float d){
        den = d;
    }
    public float getDensity(){
        return den;
    }

    public void setPressure(float p){
        pres = p;
    }
    public float getPressure(){
        return pres;
    }
    public void setForce(Vector3 f){
        force = f;
    }
    public Vector3 getForce(){
        return force;
    }
    public void setObject(GameObject o){
        obj = o;
    }

}
