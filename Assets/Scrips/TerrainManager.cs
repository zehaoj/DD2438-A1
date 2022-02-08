using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Newtonsoft.Json; // Import JSON.NET from Unity Asset store (not needed in 2022 version)


public class TerrainManager : MonoBehaviour {


    //public TestScriptNoObject testNoObject = new TestScriptNoObject();

    public string terrain_filename = "Text/terrain";
    public TerrainInfo myInfo;

    public GameObject flag;


    // Use this for initialization
    void Start()
    {

    }

    // Use this for initialization
    void Awake()
    {

        var jsonTextFile = Resources.Load<TextAsset>(terrain_filename);

        // set to false for hard coding new terrain using TerrainInfo2 below
        bool loadFromFile = true;

        if (loadFromFile)
        {
            myInfo = TerrainInfo.CreateFromJSON(jsonTextFile.text);
        }
        else
        {
            myInfo.TerrainInfo2();
            //myInfo.file_name = "test88";
            string myString = myInfo.SaveToString();
            myInfo.WriteDataToFile(myString);
        }
        myInfo.ExpandTerrain();

        myInfo.CreateCubes();

        Instantiate(flag, myInfo.start_pos, Quaternion.identity);
        Instantiate(flag, myInfo.goal_pos, Quaternion.identity);



    }



    // Update is called once per frame
    void Update () {
		
	}
}



[System.Serializable]
public class TerrainInfo
{
    public string file_name;
    public float x_low;
    public float x_high;
    public int x_N;
    public float z_low;
    public float z_high;
    public int z_N;
    public float[,] traversability;
    public float[,] expanded_traversability;
    public float x_step;
    public float z_step;
    public int nearby_x;
    public int nearby_z;

    public Vector3 start_pos;
    public Vector3 goal_pos;
    public int goal_i;
    public int goal_j;

    
    public void TerrainInfo2()
    {
        file_name = "my_terrain";
        x_low = 50f;
        x_high = 250f;
        x_N = 45;
        z_low = 50f;
        z_high = 250f;
        z_N = 7;

        start_pos = new Vector3(100f, 1f, 100f);
        goal_pos = new Vector3(110f, 1f, 110f);


        Debug.Log("Using hard coded info...");
        //traversability = new float[,] { { 1.1f, 2f }, { 3.3f, 4.4f } };
        traversability = new float[x_N, z_N]; // hardcoded now, needs to change
        for(int i = 0; i < x_N; i++)
        {
            for (int j = 0; j < z_N; j++)
            {
                if ((i == 0 || i == x_N -1) || (j == 0 || j == z_N - 1))
                {
                    traversability[i, j] = 1.0f;
                }
                else
                {
                    traversability[i, j] = 0.0f;
                }
            }
        }
    }

    public int get_i_index(float x)
    {
        int index = (int) Mathf.Floor(x_N * (x - x_low) / (x_high - x_low));
        if (index < 0)
        {
            index = 0;
        }else if (index > x_N - 1)
        {
            index = x_N - 1;
        }
        return index;

    }
    public int get_j_index(float z) // get index of given coordinate
    {
        int index = (int)Mathf.Floor(z_N * (z - z_low) / (z_high - z_low));
        if (index < 0)
        {
            index = 0;
        }
        else if (index > z_N - 1)
        {
            index = z_N - 1;
        }
        return index;
    }

    public float get_x_pos(int i)
    {
        float step = (x_high - x_low) / x_N;
        return x_low + step / 2 + step * i;
    }

    public float get_z_pos(int j) // get position of given index
    {
        float step = (z_high - z_low) / z_N;
        return z_low + step / 2 + step * j;
    }

public void ExpandTerrain()
    {
        int expand_x_ratio = (int)System.Math.Ceiling(200f / x_N);
        int expand_z_ratio = (int)System.Math.Ceiling(200f / z_N);

        float[,] ori_expanded_traversability = new float[x_N * expand_x_ratio, z_N * expand_z_ratio];
        expanded_traversability = new float[x_N * expand_x_ratio, z_N * expand_z_ratio];

        for (int i = 0; i < x_N * expand_x_ratio; i++)
        {
            for (int j = 0; j < z_N * expand_z_ratio; j++)
            {
                ori_expanded_traversability[i, j] = traversability[i / expand_x_ratio, j / expand_z_ratio];
            }
        }

        x_step = (x_high - x_low) / (x_N * expand_x_ratio);
        z_step = (z_high - z_low) / (z_N * expand_z_ratio);
        nearby_x = (int)System.Math.Ceiling(4.0f / x_step);
        nearby_z = (int)System.Math.Ceiling(4.0f / z_step);

        expanded_traversability = ori_expanded_traversability;

        // for (int i = 0; i < x_N * expand_x_ratio; i++)
        // {
        //     for (int j = 0; j < z_N * expand_z_ratio; j++)
        //     {
        //         bool not_near = true;
        //         if ((i < x_N) || (i >= (x_N - 1) * expand_x_ratio) || (j < z_N) || (j >= (z_N - 1) * expand_z_ratio)) {
        //             expanded_traversability[i, j] = 1;
        //             break;
        //         } else {
        //             for (int m = -1 * nearby_x; m < nearby_x; m++) 
        //             {
        //                 for (int n = -1 * nearby_z; n < nearby_z; n++)
        //                 {
        //                     if (ori_expanded_traversability[i + m, j + n] > 0) {
        //                         not_near = false;
        //                         break;
        //                     }
        //                 }
        //                 if (!not_near)
        //                     break;
        //             }
        //         }
        //         if (not_near) {
        //             expanded_traversability[i, j] = 1f;
        //         } else {
        //             expanded_traversability[i, j] = 0f;
        //         }
        //     }
        // }

        x_N *= expand_x_ratio;
        z_N *= expand_z_ratio;
        goal_i = get_i_index(goal_pos[0]);
        goal_j = get_j_index(goal_pos[2]);
    }

    public bool CheckObs(int i, int j)
    {
        if (expanded_traversability[i, j] > 0) {
            return true;
        } else {
            return false;
        }
    }

    public bool CheckObsAround(int i, int j)
    {
        if (expanded_traversability[i, j] > 0) {
            return true;
        }
        else if ((System.Math.Abs(i - goal_i) + System.Math.Abs(j - goal_j)) > 20) {
            for (int m = i - nearby_x / 2; m <= i + nearby_x / 2; m++)
            {
                for (int n = j - nearby_z / 2; n <= j + nearby_z / 2; n++)
                {
                    if (expanded_traversability[m, n] > 0) {
                        return true;
                    }
                }
            }
            return false;
        } else {
            return false;
        }
    }

    public float CheckObsUpandDown(int i, int j)
    {
        int up_obs_dist = 0;
        int down_obs_dist = 0;

        for (int diff = 1; diff < 2 * nearby_z; diff++)
        {
            if (expanded_traversability[i, j + diff] > 0) {
                up_obs_dist = diff;
                break;
            }
        }

        for (int diff = 1; diff < 2 * nearby_z; diff++)
        {
            if (expanded_traversability[i, j - diff] > 0) {
                down_obs_dist = diff;
                break;
            }
        }

        // no obs up or down within 3 nearby_z
        if ((up_obs_dist == 0) && (down_obs_dist == 0))
            return 0;

        if ((up_obs_dist > 0) && (down_obs_dist > 0)) {
            if ((up_obs_dist < nearby_z) && (down_obs_dist < nearby_z))
                // TODO mark this path false
                return 0;
            return ((up_obs_dist - down_obs_dist) / 2f);
            // if (down_obs_dist < nearby_z)
            //     return ((up_obs_dist - down_obs_dist) / 2f);
        }
        
        // no obs up within 3 nearby_z
        if (up_obs_dist == 0) {
            // obs down within 1 nearby_z
            // if (down_obs_dist < nearby_z)
            return (2f * nearby_z - down_obs_dist);
        } else {
            // if (up_obs_dist < nearby_z)
            return (down_obs_dist - 2f * nearby_z);
        }
    }

    public float CheckObsLeftandRight(int i, int j)
    {
        int left_obs_dist = 0;
        int right_obs_dist = 0;

        for (int diff = 1; diff < 2 * nearby_x; diff++)
        {
            if (expanded_traversability[i - diff, j] > 0) {
                left_obs_dist = diff;
                break;
            }
        }
        for (int diff = 1; diff < 2 * nearby_x; diff++)
        {
            if (expanded_traversability[i + diff, j] > 0) {
                right_obs_dist = diff;
                break;
            }
        }

        // no obs left or right within 3 nearby_x
        if ((left_obs_dist == 0) && (right_obs_dist == 0))
            return 0;

        if ((left_obs_dist > 0) && (right_obs_dist > 0)) {
            if ((left_obs_dist < nearby_z) && (right_obs_dist < nearby_z))
                // TODO mark this path false
                return 0;
            return ((right_obs_dist - left_obs_dist) / 2f);
            // if (down_obs_dist < nearby_z)
            //     return ((up_obs_dist - down_obs_dist) / 2f);
        }
        
        // no obs right within 3 nearby_x
        if (left_obs_dist > 0) {
            return (2f * nearby_x - right_obs_dist);
        } else {
            return (right_obs_dist - 2f * nearby_x);
        }

    }

    // check direction diagonal from lower left to upper right
    public int CheckObsDiagonalUp(int i, int j)
    {
        int left_down_obs_dist = 0;
        int right_up_obs_dist = 0;

        for (int diff = 1; diff < nearby_x; diff++)
        {
            if (expanded_traversability[i - diff, j - diff] > 0) {
                left_down_obs_dist = diff;
                break;
            }
        }
        for (int diff = 1; diff < nearby_x; diff++)
        {
            if (expanded_traversability[i + diff, j + diff] > 0) {
                right_up_obs_dist = diff;
                break;
            }
        }
        if ((left_down_obs_dist == 0) && (right_up_obs_dist == 0))
            return 0;
        if ((left_down_obs_dist == 0) && (right_up_obs_dist > 0))
            return (right_up_obs_dist - nearby_x);
        if ((left_down_obs_dist >= 0) && (right_up_obs_dist == 0))
            return (nearby_x - left_down_obs_dist);
        return 0;
        // TODO
        // if both have obs, redesign path
    }

    // check direction diagonal from upper left to lower right
    public int CheckObsDiagonalDown(int i, int j)
    {
        int left_up_obs_dist = 0;
        int right_down_obs_dist = 0;

        for (int diff = 1; diff < nearby_x; diff++)
        {
            if (expanded_traversability[i - diff, j + diff] > 0) {
                left_up_obs_dist = diff;
                break;
            }
        }
        for (int diff = 1; diff < nearby_x; diff++)
        {
            if (expanded_traversability[i + diff, j - diff] > 0) {
                right_down_obs_dist = diff;
                break;
            }
        }
        if ((left_up_obs_dist == 0) && (right_down_obs_dist == 0))
            return 0;
        if ((left_up_obs_dist == 0) && (right_down_obs_dist > 0))
            return (right_down_obs_dist - nearby_x);
        if ((left_up_obs_dist >= 0) && (right_down_obs_dist == 0))
            return (nearby_x - left_up_obs_dist);
        return 0;
        // TODO
        // if both have obs, redesign path
    }

    public void CreateCubes()
    {
        float x_step = (x_high - x_low) / x_N;
        float z_step = (z_high - z_low) / z_N;
        for (int i = 0; i < x_N; i++)
        {
            for (int j = 0; j < z_N; j++)
            {
                if (expanded_traversability[i, j] > 0.5f)
                {
                    GameObject cube = GameObject.CreatePrimitive(PrimitiveType.Cube);
                    cube.transform.position = new Vector3(get_x_pos(i), 0.0f, get_z_pos(j));
                    cube.transform.localScale = new Vector3(x_step, 15.0f, z_step);
                }

            }
        }
    }



    public static TerrainInfo CreateFromJSON(string jsonString)
    {
        //Debug.Log("Reading json");
        return JsonConvert.DeserializeObject<TerrainInfo>(jsonString);
        //return JsonUtility.FromJson<TerrainInfo>(jsonString);
    }

    public string SaveToString()
    {
        JsonSerializerSettings JSS = new JsonSerializerSettings()
        {
            ReferenceLoopHandling = Newtonsoft.Json.ReferenceLoopHandling.Ignore
        };

        return JsonConvert.SerializeObject(this, Formatting.None, JSS);

        //return JsonConvert.SerializeObject(this); // throws ref loop error
        //return JsonUtility.ToJson(this); // cannot handle multi-dim arrays
    }

    public void WriteDataToFile(string jsonString)
    {
        string path = Application.dataPath + "/Resources/Text/" + file_name + ".json";
        Debug.Log("AssetPath:" + path);
        System.IO.File.WriteAllText(path, jsonString);
#if UNITY_EDITOR
        UnityEditor.AssetDatabase.Refresh();
#endif
    }


}