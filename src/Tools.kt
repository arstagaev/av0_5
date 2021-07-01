import java.lang.Math.pow
import kotlin.math.*


/**
 * Определение среднего значения:
На вход метода поступает чанк данных (в размере примерно 25),
задача на выходе получить среднее значение по этому чанку
 */
fun Mean(Data_Chunk : ArrayList<XYZ>) : DoubleArray{
    // initialize first Res variable with first vector of chunk
    var Res = XYZ(Data_Chunk[0].x,Data_Chunk[0].y,Data_Chunk[0].z)

    for (i in 1 until  Data_Chunk.size){
        Res.x = Res.x + Data_Chunk[i].x
        Res.y = Res.y + Data_Chunk[i].y
        Res.z = Res.z + Data_Chunk[i].z
    }

    Res.x = Res.x / (Data_Chunk.size)
    Res.y = Res.y / (Data_Chunk.size)
    Res.z = Res.z / (Data_Chunk.size)
    println(Res.toString())

    return doubleArrayOf(Res.x,Res.y,Res.z)
}

/**
 * Определение среднеквадратического отклонения:
 *
 * На вход метода поступает чанк данных
 * и среднее значение по этому чанку
 * (не обязательно, метод может обладать полиморфизмом и,
 * если среднего значения не поступает на вход, должен высчитывать его самостоятельно).
 * На выходе – вектор – значение среднеквадратического отклонения по поступившему чанку.
 */

fun Deviation(Data_Chunk: ArrayList<XYZ>, Mean : DoubleArray) : DoubleArray{

    var Res = doubleArrayOf(0.0,0.0,0.0)

    for (i in 0 .. Data_Chunk.size) {
        Res[0] = Res[0] + pow((Data_Chunk[0].x - Mean[0]),2.0)
        Res[1] = Res[1] + pow((Data_Chunk[1].y - Mean[1]),2.0)
        Res[2] = Res[2] + pow((Data_Chunk[2].z - Mean[2]),2.0)
    }

    Res[0] = sqrt( Res[0] / Data_Chunk.size)
    Res[1] = sqrt( Res[1] / Data_Chunk.size)
    Res[2] = sqrt( Res[2] / Data_Chunk.size)
    return Res
}


/**
 * Расчет нормы вектора:
 *
 * На вход поступает вектор, задача посчитать его «длину» или же норму.
 */

fun VectorNorm(Vector : DoubleArray): Double {
    var Result = sqrt(pow(Vector[0],2.0)+pow(Vector[1],2.0)+pow(Vector[2],2.0))
    return Result
}

/**
 * Нормирование вектора:
 *
 *  На вход поступает вектор, задача привести его к нормированному виду
 *  (нормированный вид вектора имеет то же направление, что и изначальный,
 *  но при этом его длина равняется единице).
 */
fun NormVector(Vector: DoubleArray): DoubleArray {
    var Result = doubleArrayOf(0.0,0.0,0.0)
    //var a = ArrayList<Double>(5)
    var Norm = VectorNorm(Vector)
    Result[0] = Vector[0] / Norm
    Result[1] = Vector[1] / Norm
    Result[2] = Vector[2] / Norm

    return Result
}

/**
 * Скалярное произведение.
 Метод получает на вход два вектора, возвращает скалярное произведение двух векторов.
 */

fun DotProd(V1: DoubleArray, V2 : DoubleArray) : Double {

    return (V1[0]*V2[0] + V1[1]*V2[1] + V1[2]*V2[2])

}

/**
 * Векторное произведение
Метод получает на вход два вектора, возвращает векторное произведение двух векторов.
 */

fun CrossProd(V1: DoubleArray, V2: DoubleArray) : DoubleArray{
    return doubleArrayOf(
        (V1[1]*V2[2]-V1[2]*V2[1]),
        (V1[2]*V2[0]-V1[0]*V2[2]),
        (V1[0]*V2[1]-V1[1]*V2[0])
    )
}
/**
 * Расчёт поворотного вектора:
На вход подаются два вектора (внимание, для гарантии достоверного результата,
векторы лучше нормализовать заранее), на выходе четырехмерный вектор,
содержащий направление оси поворота (в первых трех элементах) и угла поворота (в четвертом).
 */
fun RotVecCalc(V1: DoubleArray, V2: DoubleArray): DoubleArray {
    var RotVec = CrossProd(V1, V2)
    var Cosang = ((DotProd(V1, V2))/(VectorNorm(V1)*VectorNorm(V2)))

    var Ang = acos(Cosang)
    RotVec[3] = Ang
    return RotVec
}

/**
 * Перевод поворотного вектора в кватернион:
На вход поступает четырехмерный поворотный вектор,
на выходе должен быть четырехмерный вектор кватерниона поворота.
 */
fun Axang2Q(RotVec : DoubleArray): DoubleArray {
    var Q = doubleArrayOf()
    var RV = doubleArrayOf()

    Q[0] = cos(RotVec[3]/2)
    RV[0] = RotVec[0]
    RV[1] = RotVec[1]
    RV[2] = RotVec[2]

    var V  = NormVector(RV)
    Q[1] = V [0] * sin(RotVec[3] / 2)
    Q[2] = V [1] * sin(RotVec[3] / 2)
    Q[3] = V [2] * sin(RotVec[3] / 2)

    return Q
}

/**
 * Перевод кватерниона в матрицу поворота:
Для перевода показаний измерителя в систему отсчета,
связанную с автомобилем, самый простой способ,
храня данные об ориентации в виде кватернионов, при вычислениях пользоваться матричным видом.
 */
fun Quat2DCM(Q : DoubleArray): Array<Array<Double>> {
    var Q = NormVector(Q)
    var DCM = arrayOf<Array<Double>>()
    DCM[0][0] = pow(Q[0],2.0)+ pow(Q[1],2.0)-pow(Q[2],2.0)-pow(Q[3],2.0)
    DCM[0][1] = 2 *(Q[1]*Q[2] + Q[0]*Q[3])
    DCM[0][2] = 2 *(Q[1]*Q[3] + Q[0]*Q[2])
    DCM[1][0] = 2 *(Q[1]*Q[2] + Q[0]*Q[3])
    DCM[1][1] = pow(Q[0],2.0)+ pow(Q[1],2.0)-pow(Q[2],2.0)-pow(Q[3],2.0)
    DCM[1][2] = 2 *(Q[2]*Q[3] + Q[0]*Q[1])
    DCM[2][0] = 2 *(Q[1]*Q[3] + Q[0]*Q[2])
    DCM[2][1] = 2 *(Q[2]*Q[3] + Q[0]*Q[1])
    DCM[2][2] = pow(Q[0],2.0)+ pow(Q[1],2.0)-pow(Q[2],2.0)-pow(Q[3],2.0)
    return DCM
}
/**
 * Поворот вектора кватернионом:
При помощи этого метода, входной вектор записывается в другой системе координат.
На вход подается вектор, который следут повернуть и соответствующий кватернион поворота.
На выходе  - вектор, повернутый кватернионом.
 */
fun Quatrotate(Q : DoubleArray,V : DoubleArray): DoubleArray {
    var DCM = Quat2DCM(Q)
    var Res = doubleArrayOf()

    Res[0] =DCM[0][0] * V[0] + DCM[0][1]*V[1] + DCM[0][2]*V[2]
    Res[1] =DCM[1][0] * V[0] + DCM[1][1]*V[1] + DCM[1][2]*V[2]
    Res[2] =DCM[2][0] * V[0] + DCM[2][1]*V[1] + DCM[2][2]*V[2]

    return Res
}

/**
 * Частное двух кватернионов:
Определяет операцию деления между двумя кватернионами (четырехмерные вектора).
На входе – два кватерниона, на выходе – результат деления между ними.
 */

fun Quatdivide(Q: DoubleArray,R : DoubleArray): DoubleArray {
    var Norm = VectorNorm(R)
    var Res = doubleArrayOf()
    Res[0] = (R[0]*Q[0] + R[1]*Q[1] + R[2]*Q[2] + R[3]*Q[3]) / Norm
    Res[1] = (R[0]*Q[1] + R[1]*Q[0] + R[2]*Q[3] + R[3]*Q[2]) / Norm
    Res[2] = (R[0]*Q[2] + R[1]*Q[3] + R[2]*Q[0] + R[3]*Q[1]) / Norm
    Res[3] = (R[0]*Q[3] + R[1]*Q[2] + R[2]*Q[1] + R[3]*Q[0]) / Norm
    return Res
}
/**
 * Нахождение угла между двумя кватернионами:
На вход поступают два кватерниона.
На выходе – угловая разница между результирующими поворотами, которые в этих кватернионах заложены.
 */
fun AngleBetweenQ(Q1 : DoubleArray, Q2 : DoubleArray): Double {
    var Q_d = Quatdivide(Q1,Q2)
    var Theta = acos(Q_d[0])*2
    return Theta
}

/**
 * Нахождение индекса максимального (по модулю) элемента в векторе:
На вход поступает вектор. На выходе – индекс максимального элемента в этом векторе.
 */
fun Find_max_index(V: DoubleArray): Int {

    var Res = 0.0
    var MAX_INDEX = 0
    //var prep = doubleArrayOf()

    for (i in 0 until V.size){
        Res = V[i]

        if (abs(V[i])> Res){
            Res = V[i]
            MAX_INDEX = i
        }
    }
    return MAX_INDEX
}

