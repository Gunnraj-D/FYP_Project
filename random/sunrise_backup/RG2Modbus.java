package application;

import java.io.*;
import java.net.*;

/** Tiny Modbus-TCP helper for an OnRobot RG2. Only: open/close. */
public class RG2Modbus {
  // === FILL THESE FROM YOUR RG2 MANUAL ===
  private static final int UNIT_ID   = 65;     // sometimes 1
  private static final int REG_CMD   = 0x0004; // command register
  private static final int CMD_OPEN  = 1;      // value for OPEN
  private static final int CMD_CLOSE = 2;      // value for CLOSE
  // =======================================

  private final String host; private final int port;
  private Socket s; private InputStream in; private OutputStream out;
  private int tid = 1;

  public RG2Modbus(String host, int port) { this.host = host; this.port = port; }

  public void connect() throws IOException {
    if (s != null && s.isConnected() && !s.isClosed()) return;
    s = new Socket();
    s.connect(new InetSocketAddress(host, port), 1500);
    s.setTcpNoDelay(true); s.setSoTimeout(1500);
    in = new BufferedInputStream(s.getInputStream());
    out = new BufferedOutputStream(s.getOutputStream());
  }

  public void close() { try { if (in!=null) in.close(); } catch (Exception ignored) {}
                       try { if (out!=null) out.close(); } catch (Exception ignored) {}
                       try { if (s!=null) s.close(); } catch (Exception ignored) {}
                       in=null; out=null; s=null; }

  public void openGripper()  throws IOException { writeSingleRegister(REG_CMD, CMD_OPEN);  }
  public void closeGripper() throws IOException { writeSingleRegister(REG_CMD, CMD_CLOSE); }

  // ---- Minimal Modbus FC6 (write single register) ----
  private void writeSingleRegister(int addr, int val) throws IOException {
    ensure();
    byte[] pdu = concat(
        be16(addr),            // address
        be16(val)              // value
    );
    send((byte)0x06, pdu);     // FC 06
    readAck();                 // ensure no exception
  }

  private void send(byte func, byte[] payload) throws IOException {
    byte[] mbap = new byte[7];
    put16(mbap, 0, tid++);     // transaction id
    put16(mbap, 2, 0);         // protocol id
    put16(mbap, 4, 1 + 1 + payload.length); // len = unit + func + payload
    mbap[6] = (byte) UNIT_ID;  // unit id
    out.write(mbap);
    out.write(func);
    out.write(payload);
    out.flush();
  }

  private void readAck() throws IOException {
    byte[] hdr = readN(7);
    int len = ((hdr[4]&0xFF)<<8) | (hdr[5]&0xFF);
    byte[] pdu = readN(len - 1);     // minus unit byte
    if ( (pdu[0] & 0x80) != 0 ) {    // exception bit
      int ex = pdu.length>1 ? (pdu[1]&0xFF) : -1;
      throw new IOException("Modbus exception: " + ex);
    }
  }

  private void ensure() throws IOException { if (s==null || s.isClosed()) connect(); }
  private static byte[] be16(int v){ return new byte[]{ (byte)(v>>8), (byte)v }; }
  private static void put16(byte[] a, int off, int v){ a[off]=(byte)(v>>8); a[off+1]=(byte)v; }
  private static byte[] concat(byte[]... parts) {
    int n=0; for (byte[] p:parts) n+=p.length; byte[] out=new byte[n]; int i=0;
    for (byte[] p:parts){ System.arraycopy(p,0,out,i,p.length); i+=p.length; }
    return out;
  }
  private byte[] readN(int n) throws IOException {
    byte[] b=new byte[n]; int off=0, r;
    while(off<n && (r=in.read(b, off, n-off))>0) off+=r;
    if (off<n) throw new EOFException("short read");
    return b;
  }
}