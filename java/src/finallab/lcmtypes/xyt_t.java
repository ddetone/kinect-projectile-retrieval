/* LCM type definition class file
 * This file was automatically generated by lcm-gen
 * DO NOT MODIFY BY HAND!!!!
 */

package finallab.lcmtypes;
 
import java.io.*;
import java.util.*;
import lcm.lcm.*;
 
public final class xyt_t implements lcm.lcm.LCMEncodable
{
    public long utime;
    public double xyt[];
    public boolean goFast;
 
    public xyt_t()
    {
        xyt = new double[3];
    }
 
    public static final long LCM_FINGERPRINT;
    public static final long LCM_FINGERPRINT_BASE = 0x5551a94bf2f07eceL;
 
    static {
        LCM_FINGERPRINT = _hashRecursive(new ArrayList<Class<?>>());
    }
 
    public static long _hashRecursive(ArrayList<Class<?>> classes)
    {
        if (classes.contains(finallab.lcmtypes.xyt_t.class))
            return 0L;
 
        classes.add(finallab.lcmtypes.xyt_t.class);
        long hash = LCM_FINGERPRINT_BASE
            ;
        classes.remove(classes.size() - 1);
        return (hash<<1) + ((hash>>63)&1);
    }
 
    public void encode(DataOutput outs) throws IOException
    {
        outs.writeLong(LCM_FINGERPRINT);
        _encodeRecursive(outs);
    }
 
    public void _encodeRecursive(DataOutput outs) throws IOException
    {
        outs.writeLong(this.utime); 
 
        for (int a = 0; a < 3; a++) {
            outs.writeDouble(this.xyt[a]); 
        }
 
        outs.writeByte( this.goFast ? 1 : 0); 
 
    }
 
    public xyt_t(byte[] data) throws IOException
    {
        this(new LCMDataInputStream(data));
    }
 
    public xyt_t(DataInput ins) throws IOException
    {
        if (ins.readLong() != LCM_FINGERPRINT)
            throw new IOException("LCM Decode error: bad fingerprint");
 
        _decodeRecursive(ins);
    }
 
    public static finallab.lcmtypes.xyt_t _decodeRecursiveFactory(DataInput ins) throws IOException
    {
        finallab.lcmtypes.xyt_t o = new finallab.lcmtypes.xyt_t();
        o._decodeRecursive(ins);
        return o;
    }
 
    public void _decodeRecursive(DataInput ins) throws IOException
    {
        this.utime = ins.readLong();
 
        this.xyt = new double[(int) 3];
        for (int a = 0; a < 3; a++) {
            this.xyt[a] = ins.readDouble();
        }
 
        this.goFast = ins.readByte()!=0;
 
    }
 
    public finallab.lcmtypes.xyt_t copy()
    {
        finallab.lcmtypes.xyt_t outobj = new finallab.lcmtypes.xyt_t();
        outobj.utime = this.utime;
 
        outobj.xyt = new double[(int) 3];
        System.arraycopy(this.xyt, 0, outobj.xyt, 0, 3); 
        outobj.goFast = this.goFast;
 
        return outobj;
    }
 
}

