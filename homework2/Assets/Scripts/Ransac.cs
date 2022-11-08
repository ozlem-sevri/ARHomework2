using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System.IO;
using System;
using Random = UnityEngine.Random;


public class Ransac : MonoBehaviour
{
    //static readonly int N = 3;
    // Number of unknowns
    static readonly  int N = 3;
    static readonly float error = 0.00001f;
    Vector3[] set1;
    Vector3[] set2;

    Vector3[] random1 ;
    Vector3[] random2 ; 

    int set1Size;
    int set2Size;

    float[,] R ;//= new float[3,3]; //Rotation matris r1,r2,r3
    float[] T ;//= new float[3]; //T matris tx,ty,tz

    private GameObject[] point1;
    private GameObject[] point2;

    //double trasheld = Math.Pow(10,-5); 

   
    // Start is called before the first frame update
    void Start()
    {
        string fileName1 = "Datas/Data.txt";
        string pathFile1 = Application.dataPath + "/" + fileName1;
        string fileName2 = "Datas/Data2.txt";
        string pathFile2 = Application.dataPath + "/" + fileName2;
        
        //set1Size = ReadFromTheFile(pathFile1,set1);
        //set2Size = ReadFromTheFile(pathFile2,set2);
        ReadFromTheFile(pathFile1,pathFile2);
        showPoints();
       // print("Set1 size: " + set1Size);
        //print("Set2 size: "+set2Size);

      
       // findRandomPositions();
       
    }

    // Update is called once per frame
    void Update()
    {
        
    }
    public void initial(int [,] arr){
        for(int i = 0; i < 3; i++)
            for(int j = 0; j < 2; j++)
                arr[i,j] = -1;
    }
    public void ReadFromTheFile(string pathfile,string pathfile2){
        
            string [] coordinates = File.ReadAllLines(pathfile);
            set1 = new Vector3[coordinates.Length - 1];
            for(int i = 1; i < coordinates.Length ; i++){
                string [] coordinate = coordinates[i].Split(" ");
                float x = float.Parse(coordinate[0]);
                float y = float.Parse(coordinate[1]);
                float z = float.Parse(coordinate[2]);
                
                set1[i-1] = new Vector3(x,y,z);
            }
            set1Size = int.Parse(coordinates[0]);

            string [] coordinates2 = File.ReadAllLines(pathfile2);
            set2 = new Vector3[coordinates2.Length - 1];
            for(int i = 1; i < coordinates2.Length ; i++){
                string [] coordinate = coordinates2[i].Split(" ");
                float x = float.Parse(coordinate[0]);
                float y = float.Parse(coordinate[1]);
                float z = float.Parse(coordinate[2]);
                
                set2[i-1] = new Vector3(x,y,z);
            }
            set2Size = int.Parse(coordinates2[0]);
            //return int.Parse(coordinates[0]); 
    }
    public bool isUnique(int[,] indexes, int i , int j){
        for(int k = 0; k < 3; k++)
                if(indexes[k,0] == i || indexes[k,1] == j)
                    return false;
        return true;


    }

    public void calculateRandT(Vector3[] source, Vector3[] target){
        Vector3 [] target_temp = new Vector3[3];
        target_temp[0] = new Vector3(target[0].x - target[1].x,target[0].y - target[1].y,target[0].z - target[1].z); //P1 - P2
        target_temp[1] = new Vector3(target[0].x - target[2].x,target[0].y - target[2].y,target[0].z - target[2].z); //P1 - P3
        target_temp[2] = new Vector3(target[1].x - target[2].x,target[1].y - target[2].y,target[1].z - target[2].z); //P2 - P3

        Vector3 [] source_temp = new Vector3[3];
        source_temp[0] = new Vector3(source[0].x - source[1].x,source[0].y - source[1].y,source[0].z - source[1].z); //A - B
        source_temp[1] = new Vector3(source[0].x - source[2].x,source[0].y - source[2].y,source[0].z - source[2].z); //A - C
        source_temp[2] = new Vector3(source[1].x - source[2].x,source[1].y - source[2].y,source[1].z - source[2].z); //B - C

        float [,] matris = new float[3,3];
        int state = 0; //R nin 0 1 2  hangi satirini dolduracagini belirt. a b c / d e f / g h j
        for(int i = 0; i < 3; i++){
            matris[i,0] = source_temp[i].x;
            matris[i,1] = source_temp[i].y;
            matris[i,2] = source_temp[i].z;
        }
         print("********MATRIS*********");
        for(int i = 0; i < 3; i++)
            for(int j = 0; j < 3; j++)
                print(matris[i,j]);
        print("********MATRIS*********");
        float [,] transpose = new float[3,3];  
        if(Determinant(matris)){
            float [,] gaussian = new float[3,4];
            for(int i = 0; i < 3; i++){
                for(int j = 0; j< 3; j++){
                    gaussian[i,j] = matris[i,j];
                }
            }
            print("Determinant is okey");
            R = new float[3,3];
            T = new float[3];

            gaussian[0,3] = target_temp[0].x;
            gaussian[1,3] = target_temp[1].x;
            gaussian[2,3] = target_temp[2].x;
            gaussianElimination(gaussian,0);
            gaussian[0,3] = target_temp[0].y;
            gaussian[1,3] = target_temp[1].y;
            gaussian[2,3] = target_temp[2].y;
            gaussianElimination(gaussian,1);
            gaussian[0,3] = target_temp[0].z;
            gaussian[1,3] = target_temp[1].z;
            gaussian[2,3] = target_temp[2].z;
            gaussianElimination(gaussian,2);

            findT();
            printRandT();
            if(calculateForRandT())
                showTransformedPoints();
            else
                findRandomPositions();
        }
          
        else if(Transpose(matris,transpose)){
            float [,] matris_transpose = new float[3,3];
            float [,] inv = new float[3,3];
            float [,] result = new float[3,3];
            matrisMultiplication(matris,transpose,matris_transpose);
            inverse(matris_transpose,inv);
            matrisMultiplication(inv,matris_transpose,result);

            for(int i = 0; i < 3; i++){
                R[i,0] = result[i,0] * random1[0].x + result[i,1] * random1[0].y + result[i,2] * random1[0].z;
            }

            

        }
            //findRandomPositions();

    
      /*  print("+++++++++++++GAUSSSIAN MATRIS+++++++++++++++");
        for(int i = 0; i < 3; i++){
            for(int j = 0; j < 4; j++)
                print(gaussian[i,j]);
            print("");
        }
        print("+++++++++++++GAUSSSIAN MATRIS+++++++++++++++");*/
        
        
       
        
    }
    bool Transpose(float[,] matris, float[,] transpose){
        for(int i = 0; i < 3; i++)
            for(int j= 0; j < 3; j++){
                transpose[i,j] = matris[j,i];
            }
        float [,] n_matris = new float[3,3]; 
        
        matrisMultiplication(matris,transpose,n_matris);
        if(Determinant(n_matris))
            return true;
        return false;
        
    }
    
    void matrisMultiplication(float[,] m, float[,] t, float[,] result){
        for(int i = 0; i < 3; i++){
            for(int j = 0; j < 3; j++){
                for (int k = 0; k < 3; k++)
                    {
                        result[i, j] += m[i, k] * t[k, j];
                    }
            }
        }
    }
    void printRandT(){
        print("************Rotation************");
        for(int i = 0; i < 3; i++)
            for(int j = 0; j < 3; j++)
                print(R[i,j]);
        print("**********T*********");
        for(int j = 0; j < 3; j++)
            print(T[j]);
    }
    public void findT(){
        float[] x = new float[3];
        for(int i = 0; i < 3; i++)
            x[i] = R[i,0] * random1[0].x + R[i,1] * random1[0].y + R[i,2] * random1[0].z;
        
        //for(int i = 0; i < 3; i++)
            T[0] = random2[0].x - x[0];
            T[1] = random2[0].y - x[1];
            T[2] = random2[0].z - x[2];
    }

    public void calculateR(float[,]inv, Vector3[] point, int state){
        for(int i = 0; i < 3; i++){
            R[state,i] = inv[i,0] * point[0].x + inv[i,1] * point[1].x + inv[i,2] * point[2].x;
        }
        print("-----------------R-----------------");
        for(int i = 0; i < 3; i++)
            for(int j = 0; j < 3; j++)
                print(R[i,j]);
        print("-----------------------------------");
    }

    void showPoints(){
        point1 = new GameObject[set1Size];
        point2 = new GameObject[set2Size];
        
        for(int i = 0; i < set1Size; i++){
            point1[i] = GameObject.CreatePrimitive(PrimitiveType.Sphere);
            point1[i].transform.position = set1[i];
            point1[i].name = "Point1_" + i;
            var sphereRenderer = point1[i].GetComponent<Renderer>();

            sphereRenderer.material.SetColor("_Color", Color.red);

        }
        for(int i = 0; i < set2Size; i++){
            point2[i] = GameObject.CreatePrimitive(PrimitiveType.Sphere);
            point2[i].transform.position = set2[i];
            point2[i].name = "Point2_" + i;
            var sphereRenderer = point2[i].GetComponent<Renderer>();

            sphereRenderer.material.SetColor("_Color", Color.black);

        }

    }
    void showTransformedPoints(){
        //showPoints();
        //int j = 0;
        for(int i = 0; i < set1Size; i++){
            float x = R[0,0] * set1[i].x + R[0,1] * set1[i].y + R[0,2] * set1[i].z;
            float y = R[1,0] * set1[i].x + R[1,1] * set1[i].y + R[1,2] * set1[i].z;
            float z = R[2,0] * set1[i].x + R[2,1] * set1[i].y + R[2,2] * set1[i].z;

            Vector3 V = new Vector3(x + T[0] , y + T[1], z + T[2]);
            if(controlTargetForV(V)){
                Debug.DrawLine(set1[i], V, Color.green, 2.5f);
            }
        }
    }

    void findRandomPositions()
    {
        int [,] indexes = new int[3,2];
        initial(indexes);
        random1 =  new Vector3[3];
        random2 =  new Vector3[3];
        int indexA;
        int indexB;
        //select random 3 points from data sets
       /* for(int i = 0; i < 3; i++){
                
         do{ 
                indexA = Random.Range(0,set1Size);
                indexB = Random.Range(0,set2Size);
            }while(!isUnique(indexes,indexA,indexB));
                
            random1[i] = set1[indexA];
            random2[i] = set2[indexB];
            indexes[i,0] = indexA;
            indexes[i,1] = indexB;


               // float currentScore = 0;
               // List<Vector3> currentPoints = new List<Vector3>();   
        }*/
        random1[0] = set1[0];
        random1[1] = set1[1];
        random1[2] = set1[2];
        random2[0] = set2[0];
        random2[1] = set2[1];
        random2[2] = set2[2];
        print("*****Random coordinates*****");
        foreach(Vector3 r in random1){
            print("Random 1");
            print(r);
        }
        foreach(Vector3 r in random2){
            print("Random 2");
            print(r);
        }
        print("-------------");
        calculateRandT(random1,random2);
    }

    bool calculateForRandT(){
        int total = 0;
         Vector3 V;
        for(int i = 0; i < set1Size; i++){
            float x = R[0,0] * set1[i].x + R[0,1] * set1[i].y + R[0,2] * set1[i].z;
            float y = R[1,0] * set1[i].x + R[1,1] * set1[i].y + R[1,2] * set1[i].z;
            float z = R[2,0] * set1[i].x + R[2,1] * set1[i].y + R[2,2] * set1[i].z;

            V = new Vector3(x + T[0] , y + T[1], z + T[2]);
            if(controlTargetForV(V))
                total++;

        }
        if(total >= (set1Size  ))
            return true;
        return false;
    }

    bool controlTargetForV(Vector3 v){
        for(int i = 0; i < set2Size; i++)
            if(v.x == set2[i].x && v.y == set2[i].y && v.y == set2[i].z)
                return true;
        return false;
    }

    bool Determinant(float [,] matris){
        float det = 0;

        for(int i=0;i<3;i++)
            det = det + (matris[0,i]*(matris[1,(i+1)%3]*matris[2,(i+2)%3] - matris[1,(i+2)%3]*matris[2,(i+1)%3]));
        print("determinant : " + det);
        if(det < 0)
        det *= -1; 
        if(det  > error)
            return true;
        return false;
    }



// Function to get matrix content
  void gaussianElimination(float [,]mat,int state)
{

	/* reduction into r.e.f. */
	int singular_flag = forwardElim(mat);
	
	/* if matrix is singular */
	if (singular_flag != -1)
	{
		print("Singular Matrix.");
		
		/* if the RHS of equation corresponding to
		zero row is 0, * system has infinitely
		many solutions, else inconsistent*/
		if (mat[singular_flag,N] != 0)
			print("Inconsistent System.");
		else
			print("May have infinitely " +
						"many solutions.");
		
		return;
	}
	
	/* get solution to system and print it using
	backward substitution */
	backSub(mat,state);
}

// Function for elementary operation of swapping two
// rows
 void swap_row(float [,]mat, int i, int j)
{
	
	// printf("Swapped rows %d and %d\n", i, j);
	
	for(int k = 0; k <= N; k++)
	{
		float temp = mat[i, k];
		mat[i, k] = mat[j, k];
		mat[j, k] = temp;
	}
}
	
// Function to print matrix content at any stage
static void print(float [,]mat)
{
    print("+++++++++++++++++++++++++++++++++");
	for(int i = 0; i < N; i++ )
		for(int j = 0; j <= N; j++)
			print(mat[i, j]);
			
	print("+++++++++++++++++++++++++++++++++");
   
    
}

// Function to reduce matrix to r.e.f.
 int forwardElim(float [,]mat)
{
	for(int k = 0; k < N; k++)
	{
		
		// Initialize maximum value and index for pivot
		int i_max = k;
		int v_max = (int)mat[i_max, k];
		
		/* find greater amplitude for pivot if any */
		for(int i = k + 1; i < N; i++)
		{
			if (Math.Abs(mat[i, k]) > v_max)
			{
				v_max = (int)mat[i, k];
				i_max = i;
			}
		
			/* If a principal diagonal element is zero,
			* it denotes that matrix is singular, and
			* will lead to a division-by-zero later. */
			if (mat[k, i_max] == 0)
				return k; // Matrix is singular
			
			/* Swap the greatest value row with
			current row
			*/
			if (i_max != k)
				swap_row(mat, k, i_max);
			
			for(int j = k + 1; j < N; j++)
			{
			
				/* factor f to set current row kth element
				* to 0, and subsequently remaining kth
				* column to 0 */
				float f = mat[j, k] / mat[k, k];
				
				/* subtract fth multiple of corresponding
				kth row element*/
				for(int t = k + 1; t <= N; t++)
					mat[j, t] -= mat[k, t] * f;
				
				/* filling lower triangular matrix with
				* zeros*/
				mat[j, k] = 0;
			}
		}
		// print(mat);	 //for matrix state
	}
	
	// print(mat);		 //for matrix state
	return -1;
}

// Function to calculate the values of the unknowns
 void backSub(float [,]mat,int state)
{
	
	// An array to store solution
	float []x = new float[N];
	
	/* Start calculating from last equation up to the
	first */
	for(int i = N - 1; i >= 0; i--)
	{
	
		/* start with the RHS of the equation */
		x[i] = mat[i,N];
		
		/* Initialize j to i+1 since matrix is upper
		triangular*/
		for(int j = i + 1; j < N; j++)
		{
		
			/* subtract all the lhs values
			* except the coefficient of the variable
			* whose value is being calculated */
			x[i] -= mat[i,j] * x[j];
		}
		
		/* divide the RHS by the coefficient of the
		unknown being calculated */
		x[i] = x[i] / mat[i,i];
	}
	
	print("Solution for the system:");
	for(int i = 0; i < 3; i++)
	{
        /*if(Double.IsNaN((double)x[i])){
            findRandomPositions();
            return;
        }*/
        R[state,i] = x[i];
		print( x[i]);
		
	}
}
// Function to get cofactor of A[p,q] in [,]temp. n is current
// dimension of [,]A
static void getCofactor(float [,]A, float [,]temp, int p, int q, int n)
{
	int i = 0, j = 0;

	// Looping for each element of the matrix
	for (int row = 0; row < n; row++)
	{
		for (int col = 0; col < n; col++)
		{
			// Copying into temporary matrix only those element
			// which are not in given row and column
			if (row != p && col != q)
			{
				temp[i, j++] = A[row, col];

				// Row is filled, so increase row index and
				// reset col index
				if (j == n - 1)
				{
					j = 0;
					i++;
				}
			}
		}
	}
}

/* Recursive function for finding determinant of matrix.
n is current dimension of [,]A. */
static float determinant(float [,]A, int n)
{
	float D = 0; // Initialize result

	// Base case : if matrix contains single element
	if (n == 1)
		return A[0, 0];

	float [,]temp = new float[N, N]; // To store cofactors

	int sign = 1; // To store sign multiplier

	// Iterate for each element of first row
	for (int f = 0; f < n; f++)
	{
		// Getting Cofactor of A[0,f]
		getCofactor(A, temp, 0, f, n);
		D += sign * A[0, f] * determinant(temp, n - 1);

		// terms are to be added with alternate sign
		sign = -sign;
	}
	return D;
}

// Function to get adjoint of A[N,N] in adj[N,N].
static void adjoint(float [,]A, float [,]adj)
{
	if (N == 1)
	{
		adj[0, 0] = 1;
		return;
	}

	// temp is used to store cofactors of [,]A
	int sign = 1;
	float [,]temp = new float[N, N];

	for (int i = 0; i < N; i++)
	{
		for (int j = 0; j < N; j++)
		{
			// Get cofactor of A[i,j]
			getCofactor(A, temp, i, j, N);

			// sign of adj[j,i] positive if sum of row
			// and column indexes is even.
			sign = ((i + j) % 2 == 0)? 1: -1;

			// Interchanging rows and columns to get the
			// transpose of the cofactor matrix
			adj[j, i] = (sign) * (determinant(temp, N - 1));
		}
	}
}

// Function to calculate and store inverse, returns false if
// matrix is singular
static bool inverse(float [,]A, float [,]inverse)
{
   
	// Find determinant of [,]A
	float det = determinant(A, N);
    print("detection error : " +det+ " "+ error);
    if(det < error){
        print("error in some elements when calculating the dif");
        det = 0;
    }
    print("Determinant "+det);
    
     if (det == 0)
	{
		print("Singular matris ,can't find its inverse");
        return false;
		
	}
    
	// Find adjoint
	float [,]adj = new float[N, N];
	adjoint(A, adj);

	// Find Inverse using formula "inverse(A) = adj(A)/det(A)"
	for (int i = 0; i < N; i++)
		for (int j = 0; j < N; j++)
			inverse[i, j] = adj[i, j]/det;
    return true;

}

}
