
__inline__ __device__ void set(const uint3 & pos, short2* data, uint3 size, const float2 & d )
{
    //data[pos.x + pos.y * size.x + pos.z * size.x * size.y] = fromFloat(d);
    atomicExch((int*)&data[pos.x + pos.y * size.x + pos.z * size.x * size.y], *((int*)&fromFloat(d)));
}