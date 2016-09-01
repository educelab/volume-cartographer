int main(int argc, char** argv)
{
    int* arr = new int[5];
    delete[] arr;

    // Access OOB memory. ASAN should catch this
    return arr[7];
}
