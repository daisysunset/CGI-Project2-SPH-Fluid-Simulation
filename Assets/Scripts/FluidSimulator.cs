using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class FluidSimulator : MonoBehaviour
{
    //Constants
    const float GAS = -40.0f;
    const float GRAV = 1.0f;
    const float VISC = 0.1f;
    
    //Simulation Attributes
    private GameObject cup;
    private GameObject cup2;
    private GameObject cup3;
    private GameObject[] colliderList1;
    private GameObject[] colliderList2;
    private Particle[] particles;
    private int cNum1;   //Number of colliders in cup
    private int cNum2;
    private float restDensity;
    private Vector3[] cup_position;
    private GameObject cur_cup;

    //Changeable attributes
    public int pNum = 400;    //Number of particles
    public int pNumMin = 100; //Number of particles
    public int pNumActive;
    public Vector3 g;   //Gravity
    public float r;     //Particle radius
    public float m;     //Particle mass
    public float ks;    //Spring coefficient
    public float kd;    //Drag coefficient
    public float smoothRad; //Smoothing Radius
    public float poly_coef1;
    public float poly_coef2;
    public bool move;

    //Attributes
    public float[] density;
    //public float[] dist;
    //public Vector3[] dir;

    // Start is called before the first frame update
    void Start()
    {   
        cup = GameObject.Find("Cup");
        cup2 = GameObject.Find("Cup2");
        cup3 = GameObject.Find("Cup3");
        cur_cup = cup;
        //Find colliders by tag
        colliderList1 = GameObject.FindGameObjectsWithTag("sph_collider");
        colliderList2 = GameObject.FindGameObjectsWithTag("sph_collider_sp");
        cNum1 = colliderList1.Length;
        cNum2 = colliderList2.Length;
        cup_position = new Vector3[3];
        cup_position[0] = cup.transform.position;
        cup_position[1] = cup2.transform.position;
        cup_position[2] = cup3.transform.position;

        pNumActive = 0;
        restDensity = 0;
        poly_coef1 = m * 315.0f / (64.0f * Mathf.PI * Mathf.Pow(smoothRad, 9.0f));
        poly_coef2 = m * (-45.0f / (Mathf.PI * Mathf.Pow(smoothRad, 6.0f)));

        //Create particle array
        particles = new Particle[pNum];
        density = new float[pNum];
        //dist = new float[pNum];
        //dir = new Vector3[pNum];
    }

    private void initParticles(Vector3 tmp){   //**Create and initialize particles
        GameObject particle_prefab = (GameObject)Resources.Load("Water_Particle");
        
        //Instantiate particle object
        GameObject go  = Instantiate(particle_prefab);
        particles[pNumActive] = go.AddComponent<Particle>();
        particles[pNumActive].setObject(go);

        //Generate start position
        float x = Random.Range(tmp[0]-0.5f, tmp[0]+0.5f);
        float z = Random.Range(tmp[2]-0.5f, tmp[2]+0.5f);
        Vector3 p0 = new Vector3(x, tmp[1], z);
        
        particles[pNumActive].setRadius(r);
        particles[pNumActive].setPosition(p0);

        pNumActive++;
        restDensity += m;
    }
    

    private void updateCollidersR(Vector3 rotate_angle){
        //Rotate cup
        rotate_angle += cur_cup.transform.rotation.eulerAngles;
        cur_cup.transform.rotation = Quaternion.Euler(rotate_angle);
    }

    private void updateCollidersP(Vector3 delta_pos){
        //Move cup
        cur_cup.transform.position += delta_pos;
    }

    void keyboard_control(){
        //Press to Rotate or move the Cup[when there are some particles]
        if (pNumActive >= pNumMin){
            if(Input.GetKey(KeyCode.Z))
                updateCollidersR(new Vector3(0.0f, 0.0f, -0.5f));
            else if(Input.GetKey(KeyCode.C))
                updateCollidersR(new Vector3(0.0f, 0.0f, 0.5f));                

            if(Input.GetKey(KeyCode.W))
                updateCollidersP(new Vector3(0.0f, 0.01f, 0.0f));
            else if(Input.GetKey(KeyCode.S))
                updateCollidersP(new Vector3(0.0f, -0.01f, 0.0f));
            else if(Input.GetKey(KeyCode.A))
                updateCollidersP(new Vector3(0.0075f, 0.0f, 0.0f));
            else if(Input.GetKey(KeyCode.D))
                updateCollidersP(new Vector3(-0.0075f, 0.0f, 0.0f));  

            if(Input.GetKey(KeyCode.X)){
                cur_cup.transform.rotation = Quaternion.Euler(30.0f * Mathf.Cos(Time.time), 0,30.0f * Mathf.Sin(Time.time));
            }
            
            // Reset Cup only
            if(Input.GetKey(KeyCode.T)){
                cup.transform.position = cup_position[0];
                cup.transform.rotation = Quaternion.Euler(new Vector3(0.0f,0.0f,0.0f));
                cup2.transform.position = cup_position[1];
                cup2.transform.rotation = Quaternion.Euler(new Vector3(0.0f,0.0f,0.0f));
                cup3.transform.position = cup_position[2];
                cup3.transform.rotation = Quaternion.Euler(new Vector3(0.0f,-90.0f,0.0f));
            }
            // Reset Cup and Water
            if(Input.GetKey(KeyCode.R)){
                cup.transform.position = cup_position[0];
                cup.transform.rotation = Quaternion.Euler(new Vector3(0.0f,0.0f,0.0f));
                cup2.transform.position = cup_position[1];
                cup2.transform.rotation = Quaternion.Euler(new Vector3(0.0f,0.0f,0.0f));
                cup3.transform.position = cup_position[2];
                cup3.transform.rotation = Quaternion.Euler(new Vector3(0.0f,-90.0f,0.0f));
                cur_cup = cup;
                for(int i = 0;i < pNumActive; i++){
                    Destroy(particles[i].obj);
                    particles[i] = null;
                }
                pNumActive = 0;
                restDensity = 0;
            }
        }
          
        //Press 1 to spawn particles[until maxNum]
        if(pNumActive < pNum){
            if(pNumActive < pNumMin){
                initParticles(cur_cup.transform.position + new Vector3(0.0f,1.0f,0.0f));
            }
            if(Input.GetKey(KeyCode.Alpha1))
                initParticles(cur_cup.transform.position + cur_cup.transform.rotation * new Vector3(0.0f,1.0f,0.0f));
        }
        /*if(Input.GetKeyDown(KeyCode.Alpha2)){
            cur_cup = cup2;
            print("change to cup2");
        }
        if(Input.GetKeyDown(KeyCode.Alpha3)){
            cur_cup = cup3;
            print("change to cup3");   
        }*/

    }
    
    // FixedUpdate is called per fixed seconds
    void FixedUpdate(){
        float dt = Time.fixedDeltaTime;

        calcDensityPressure();
        
        //Calculate total force acting on each particle
        for(int i = 0; i < pNumActive; i++){
            Vector3 Fg = Vector3.zero, Fp = Vector3.zero, Fv = Vector3.zero;

            //Calculate force of gravity
            Fg = GRAV * g * particles[i].getDensity();
            //Calculate force of each neighboring particle
            for(int j = 0; j < pNumActive; j++){
                if(i == j)
                    continue;

                Vector3 dir = particles[j].getPosition() - particles[i].getPosition();
                float dist = dir.magnitude; 

                if(dist < smoothRad){
                    //Calculate force of pressure -- Spikey gradient
                    Fp += -1.0f * dir.normalized  * (particles[i].getPressure() + particles[j].getPressure()) / (2.0f * particles[j].getDensity()) 
                    * poly_coef2 * Mathf.Pow(smoothRad - dist, 2.0f);
                    //Calculate viscocity force -- Viscosity Laplacian
                    Fv += VISC  * (particles[i].getVelocity() - particles[j].getVelocity()) / particles[j].getDensity() * -poly_coef2 * (smoothRad - dist);
                }
            }
            
            Vector3 Ftotal = Fp + Fg + Fv;
            particles[i].setForce(Ftotal);
        }
        
        //Set new positions
        moveParticles(dt);

        keyboard_control();
    }

    private void calcDensityPressure(){     
        for(int i = 0; i < pNumActive; i++)
            density[i] = 0;
        for(int i = 0; i < pNumActive; i++){
            for(int j = i + 1; j < pNumActive; j++){
                float dist = (particles[j].getPosition() - particles[i].getPosition()).magnitude;
                //Poly6 kernel
                if(dist < smoothRad){
                    float tmp = poly_coef1 * Mathf.Pow(smoothRad - dist, 3.0f);
                    density[i] += tmp;
                    density[j] += tmp;
                }
            }
            particles[i].setDensity(Mathf.Max(density[i], restDensity));
            particles[i].setPressure(GAS*(particles[i].getDensity()-restDensity));
        }
    }

    private void moveParticles(float dt){   
        for(int i = 0; i < pNumActive; i++){
            Particle p = particles[i];
            //Get initial position and velocity
            Vector3 iVel = p.getVelocity(), iPos = p.getPosition();
            Vector3 vel, pos;

            //Calc velocity
            Vector3 a = p.getForce()/p.getDensity();
            vel = iVel + (a * dt);
            
            //Check for collision against every collider
            for(int j = 0; j < cNum1; j++){
                Vector3 cNormal;
                if( isColliding(iPos, vel, colliderList1[j], out cNormal) ){
                    //Resolve collision velocity
                    vel = resolveCollision(colliderList1[j], vel, cNormal);
                    //Penalty
                    iPos += (r/20.0f * cNormal);
                }
            }
            for(int j = 0; j < cNum2; j++){
                Vector3 cNormal;
                if( isColliding2(iPos, vel, colliderList2[j], out cNormal) ){
                    //print("cylinder collide");
                    //Resolve collision velocity
                    vel = resolveCollision2(colliderList2[j], vel, cNormal);
                    //Penalty
                    iPos += (r/20.0f * cNormal);
                }
            }
            
            //Calc position
            pos = iPos + (vel * dt);

            //Set vel and pos in particle
            p.setVelocity(vel);
            p.setPosition(pos);
        }

    }

    private bool isColliding(Vector3 pos, Vector3 vel, GameObject col, out Vector3 cNormal){
        // 垂直于障碍物平面朝里的方向
        cNormal = col.transform.up;
        // 障碍物里面中心坐标
        Vector3 col_pos = col.transform.position - col.transform.localScale.y/2 * cNormal;
        // 球心到障碍物中心的向量(指向球心)
        Vector3 disVec = pos - col_pos;
        // 球心到障碍物平面的距离(有正负)
        float dist = Vector3.Dot(disVec, cNormal);

        //范围判断
        //圆盘[假设只会正面朝上]
        if(col.name == "ground"){
            float rad = col.transform.localScale.x/2;
            float rad_dist = (disVec - dist * cNormal).magnitude;
            if(rad_dist + r/2 > rad ) return false;
            //print("ground collide");
        }
        //长方体
        else{
            float right_wid = col.transform.localScale.x/2;
            float dist_right = Mathf.Abs(Vector3.Dot(disVec,col.transform.right));
            float forward_wid = col.transform.localScale.z/2;
            float dist_forward = Mathf.Abs(Vector3.Dot(disVec,col.transform.forward));
            if(dist_right > right_wid || dist_forward > forward_wid) return false; 
        }

        // 球心在里面 and 贴近壁 and 速度方向是撞上去的
        if(dist > 0.0f && (dist- r/2.0f) < 0.05f && Vector3.Dot(vel,cNormal) < 0) return true;
        return false;
    }

    private bool isColliding2(Vector3 pos, Vector3 vel, GameObject col, out Vector3 cNormal){
        Vector3 col_pos = col.transform.position;
        // 记录宽度 默认x=z
        float rad = col.transform.localScale.x/2;
        // 同一平面内的圆心
        Vector3 tmp_pos = new Vector3(col_pos[0],pos[1],col_pos[2]);
        // 计算距离
        float dist = rad - (tmp_pos - pos).magnitude;

        cNormal = (tmp_pos - pos).normalized;

        // 先判断高度
        float up_wid = col.transform.localScale.y;
        float dis_up = Mathf.Abs(col_pos[1] - pos[1]);
        if (dis_up > up_wid) return false;

        // 球心在里面 and 贴近壁 and 速度方向是撞上去的
        if(dist > 0.0f && dist - (r/2.0f) < 0.05f && Vector3.Dot(vel,cNormal) < 0) return true;
        return false;
    }

    private Vector3 resolveCollision(GameObject col, Vector3 vel, Vector3 cNormal){     
        Vector3 adj = Vector3.Cross(col.transform.up, col.transform.right);

        Vector3 nVel = (ks * Vector3.Dot(vel, cNormal) * cNormal) 
        + ((1.0f - kd) * Vector3.Dot(vel, col.transform.right) * col.transform.right)
        + ((1.0f - kd) * Vector3.Dot(vel, adj) * adj);
        return nVel;
    }

    private Vector3 resolveCollision2(GameObject col, Vector3 vel, Vector3 cNormal){
        Vector3 adj = Vector3.Cross(cNormal, Vector3.up);

        Vector3 nVel = (ks * Vector3.Dot(vel, cNormal) * cNormal) 
        + ((1.0f - kd) * Vector3.Dot(vel, Vector3.up) * Vector3.up)
        + ((1.0f - kd) * Vector3.Dot(vel, adj) * adj);
        return nVel;
    }
}
