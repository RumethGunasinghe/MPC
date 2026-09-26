lock.lock();
try {
    instanceData++;
    classData++;
}
finally {
    lock.unlock();
}
