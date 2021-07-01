import kotlin.math.abs
import kotlin.math.sign
import kotlin.math.sqrt

fun main(){
    Algorithm().calc()
}



class Algorithm {
    var Events = arrayListOf<EventLine>()

    var arrayOfXYZ = arrayListOf<XYZ>()

    var isHorisonted = 0
    var isAzimuted = 0
    //var STOP_THRESHOLD=0.0002; //Do adaptive
    var GAS_THRESHHOLD=0.06;   //Do adaptive
    //var GAS_BREAKS_THRESHHOLD = 0 // really?
    val Chunk_size = 25       // Constant?

    var first_chunk_completed=-1;
    var pos=0;
    var isLocatedVert=false;
    var isLocatedForw=false;

    var Grav: IntArray = intArrayOf(0, 0, 0);
    //var Forw: IntArray = intArrayOf(0, 0, 0);
    var Forw = 0
    var Hor_counter=0;
    var Forw_counter=0;
    var Q_Hor       = doubleArrayOf(1.0, 0.0 ,0.0 ,0.0)
    var Q_Hor_old   = doubleArrayOf(1.0, 0.0 ,0.0 ,0.0)
    var Q_Forw      = doubleArrayOf(1.0, 0.0 ,0.0 ,0.0)
    var GB_s=0;
    var GB_f=0;
    var GB_event=false;
    var isReverse=1;
    var Side = -1
    //Preallocating arrays
//    condition=zeros(size(T,1),1);
//    MeanA=zeros(size(T,1),3);
//    Chunk_A=zeros(Chunk_size,3);
//    Chunk_F=zeros(Chunk_size,3);
//    Events_M=zeros(size(T,1),5);
    var MeanA = doubleArrayOf()
    var MeanA_mod = doubleArrayOf()
    var RMS_amplitude = doubleArrayOf()
    var RMS_Mean = doubleArrayOf()
    var DevA = doubleArrayOf()
    var DevA_mod : Double = 0.0

    var NorMean = doubleArrayOf()
    var Vert : Int = 0
    var Axang_Hor = doubleArrayOf()
    var ANGLE_CRITERIA = 0.0
    var IS_HORISONTED = 0
    //var TURN_THRESHHOLD = 0

    //1
    var Suspect = doubleArrayOf()
    var A1 = doubleArrayOf(0.0, 0.0, 0.0)
    var Axang_forw = doubleArrayOf()
    var Q_Forw_old      = doubleArrayOf(1.0, 0.0 ,0.0 ,0.0)
    var IS_AZIMUTED = 0

    var STOP_THRESHOLD =0.05
    var GAS_BREAKS_THRESHHOLD =0.06
    var TURN_THRESHHOLD = 0.08
    var BUMP_THRESHHOLD = 0.2

    fun calc(){
        for (i in 0 .. arrayOfXYZ.size){
            var Chunk_A = arrayOfXYZ // vvariables 25

            pos=pos+1;
            if(pos>Chunk_size){
                pos=1;
            }
            if(i>=Chunk_size){
                first_chunk_completed=0;
            }

            var CONDITION = first_chunk_completed + isHorisonted + isAzimuted
            var GB_int = 0
            var GB_fin = 0

            when(CONDITION){
                -1 -> { println("###################### CONDITION = -1 ##############################") }
                0 -> {
                    MeanA = Mean(Chunk_A)
                    MeanA_mod = NormVector(MeanA)
                    DevA = Deviation(Chunk_A,MeanA)
                    DevA_mod = VectorNorm(DevA)
                    if (DevA_mod < STOP_THRESHOLD){
                        //Events
                        NorMean = NormVector(MeanA)
                        var Grav = doubleArrayOf(0.0, 0.0, 0.0)
                        Vert = Find_max_index(NorMean)
                        Grav[Vert] = 1.0
                        Axang_Hor = RotVecCalc(Grav,NorMean)
                        Q_Hor_old = Q_Hor
                        Q_Hor = Axang2Q(Axang_Hor)


                        if (AngleBetweenQ(Q_Hor,Q_Hor_old) < ANGLE_CRITERIA){
                            Hor_counter++
                            if (Hor_counter>Chunk_size/2){
                                IS_HORISONTED = 1
                            }
                        }

                    }



                }
                1 -> {
                    MeanA = Mean(Chunk_A)
                    DevA  = Deviation(Chunk_A,MeanA)
                    DevA_mod = VectorNorm(DevA)
                    MeanA = Quatrotate(Q_Hor,MeanA)
                    DevA  = Quatrotate(Q_Hor,MeanA)

                    MeanA[Vert] = 0.0
                    if(VectorNorm(MeanA) > GAS_BREAKS_THRESHHOLD){
                        Suspect= MeanA
                        Forw = Find_max_index(Suspect)

                        A1[Forw] = 1.0
                        Side = 3 - (Vert+Forw)
                    }

                    if( DevA_mod < STOP_THRESHOLD ){
                        //Events(t,0) = 1
                        Axang_forw = RotVecCalc(A1,NormVector(Suspect))
                        Q_Forw_old = Q_Forw
                        Q_Forw = Axang2Q(Axang_forw)

                        if (AngleBetweenQ(Q_Forw,Q_Forw_old) < ANGLE_CRITERIA){
                            Forw_counter++
                            if( Forw_counter > Chunk_size / 5 ){
                                IS_AZIMUTED = 1
                            }
                        }
                    }

                }
                2 -> {
                    MeanA = Mean(Chunk_A)
                    DevA = Deviation(Chunk_A, MeanA)
                    DevA_mod = VectorNorm(DevA)

                    MeanA = Quatrotate(Q_Hor, MeanA)
                    DevA  = Quatrotate(Q_Hor,DevA)
                    MeanA = Quatrotate(Q_Forw,MeanA)
                    DevA  = Quatrotate(Q_Forw,DevA)

                    if( abs(MeanA[Forw]) > GAS_BREAKS_THRESHHOLD ) {

                        Events.add( EventLine(0,((MeanA[Forw]).toInt().sign*isReverse),0,0) )

                    }

                    if (DevA_mod < STOP_THRESHOLD) {
                        Events.add( EventLine(1,0,0,0) )
                    }
                    if (abs(MeanA[Side])>TURN_THRESHHOLD) {
                        Events.add( EventLine(1,0,((MeanA[Forw]).toInt().sign*isReverse),0) )
                    }

                    if(DevA[Vert] > BUMP_THRESHHOLD){
                        Events.add( EventLine(0,0,0,1) )

                    }

                }
            }

        }
    }
}