package com.ritwik.neuromorphic;

import android.app.Activity;
import android.app.AlertDialog;
import android.graphics.Canvas;
import android.graphics.Paint;
import android.graphics.Path;
import android.graphics.Typeface;
import android.graphics.Color;
import android.os.Bundle;
import android.os.Handler;
import android.os.Looper;
import android.content.Context;
import android.content.SharedPreferences;
import android.view.Gravity;
import android.view.MotionEvent;
import android.view.View;
import android.widget.Button;
import android.widget.EditText;
import android.widget.FrameLayout;
import android.widget.HorizontalScrollView;
import android.widget.LinearLayout;
import android.widget.ScrollView;
import android.widget.TextView;
import android.widget.Toast;
import org.json.JSONArray;
import org.json.JSONObject;
import java.io.BufferedReader;
import java.io.InputStreamReader;
import java.net.HttpURLConnection;
import java.net.URL;
import java.net.URLEncoder;
import java.nio.charset.StandardCharsets;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;

public class MainActivity extends Activity {
    private static final String BASE = "http://192.168.4.1";
    private static final int BG = Color.rgb(10, 15, 29), PANEL = Color.rgb(20, 28, 46), PANEL2 = Color.rgb(27, 38, 59);
    private static final int TEXT = Color.rgb(239, 244, 255), MUTED = Color.rgb(153, 169, 194), MINT = Color.rgb(105, 231, 193), RED = Color.rgb(255, 105, 125), AMBER = Color.rgb(255, 202, 112), BLUE = Color.rgb(105, 155, 255);
    private final Handler handler = new Handler(Looper.getMainLooper());
    private final ExecutorService net = Executors.newFixedThreadPool(2);
    private volatile boolean fetching = false;
    private JSONObject state = new JSONObject();
    private String tab = "Home";
    private LinearLayout root, body, nav;
    private TextView connection, pageTitle;
    private MapView mapView;
    private boolean polling = false;
    private long lastReportMs = 0;
    private SharedPreferences prefs;

    @Override public void onCreate(Bundle b) {
        super.onCreate(b);
        getWindow().setStatusBarColor(BG); getWindow().setNavigationBarColor(BG);
        prefs = getSharedPreferences("robot_reports", MODE_PRIVATE);
        buildShell(); renderTab(); startPolling();
    }

    private void buildShell() {
        root = new LinearLayout(this); root.setOrientation(LinearLayout.VERTICAL); root.setBackgroundColor(BG);
        LinearLayout header = new LinearLayout(this); header.setOrientation(LinearLayout.VERTICAL); header.setPadding(dp(20),dp(15),dp(20),dp(12)); header.setBackgroundColor(BG);
        LinearLayout top = new LinearLayout(this); top.setGravity(Gravity.CENTER_VERTICAL);
        TextView logo = new TextView(this); logo.setText("◉"); logo.setTextColor(MINT); logo.setTextSize(25); logo.setPadding(0,0,dp(10),0); top.addView(logo);
        LinearLayout titles = new LinearLayout(this); titles.setOrientation(LinearLayout.VERTICAL); titles.setLayoutParams(new LinearLayout.LayoutParams(0,-2,1));
        TextView brand = new TextView(this); brand.setText("NEUROMORPHIC"); brand.setTextColor(MUTED); brand.setTextSize(10); brand.setLetterSpacing(.16f); brand.setTypeface(null,Typeface.BOLD); titles.addView(brand);
        pageTitle = new TextView(this); pageTitle.setText("Robot overview"); pageTitle.setTextColor(TEXT); pageTitle.setTextSize(22); pageTitle.setTypeface(null,Typeface.BOLD); titles.addView(pageTitle); top.addView(titles);
        connection = new TextView(this); connection.setText("● CONNECTING"); connection.setTextColor(AMBER); connection.setTextSize(10); connection.setTypeface(null,Typeface.BOLD); top.addView(connection); header.addView(top);
        root.addView(header);
        FrameLayout contentFrame = new FrameLayout(this); contentFrame.setId(View.generateViewId()); contentFrame.setLayoutParams(new LinearLayout.LayoutParams(-1,0,1));
        body = new LinearLayout(this); body.setOrientation(LinearLayout.VERTICAL); contentFrame.addView(body); root.addView(contentFrame);
        nav = new LinearLayout(this); nav.setOrientation(LinearLayout.HORIZONTAL); nav.setBackgroundColor(PANEL); nav.setPadding(dp(4),dp(7),dp(4),dp(7)); root.addView(nav);
        setContentView(root); buildNav();
    }

    private void buildNav() {
        nav.removeAllViews(); String[] names={"Home","Map","Drive","Learning","Settings"}; String[] icons={"⌂","⌘","↕","✳","⚙"};
        for(int i=0;i<names.length;i++){ final String name=names[i]; LinearLayout item=new LinearLayout(this); item.setOrientation(LinearLayout.VERTICAL); item.setGravity(Gravity.CENTER); item.setPadding(0,dp(4),0,dp(4)); item.setBackground(makeBg(tab.equals(name)?PANEL2:Color.TRANSPARENT,12));
            TextView icon=new TextView(this);icon.setText(icons[i]);icon.setTextSize(19);icon.setTextColor(tab.equals(name)?MINT:MUTED);icon.setGravity(Gravity.CENTER);item.addView(icon);
            TextView label=new TextView(this);label.setText(name);label.setTextSize(10);label.setTextColor(tab.equals(name)?TEXT:MUTED);label.setGravity(Gravity.CENTER);item.addView(label);
            item.setLayoutParams(new LinearLayout.LayoutParams(0,-2,1));item.setOnClickListener(v->{tab=name;buildNav();renderTab();});nav.addView(item);
        }
    }

    private void renderTab() {
        body.removeAllViews(); pageTitle.setText(tab.equals("Home")?"Robot overview":tab.equals("Map")?"Internal map":tab.equals("Drive")?"Drive & navigate":tab.equals("Learning")?"Learning & trips":"Settings");
        ScrollView scroll=new ScrollView(this);scroll.setFillViewport(true); LinearLayout page=new LinearLayout(this);page.setOrientation(LinearLayout.VERTICAL);page.setPadding(dp(16),dp(2),dp(16),dp(20));scroll.addView(page);body.addView(scroll,new LinearLayout.LayoutParams(-1,-1));
        switch(tab){case "Home":home(page);break;case "Map":map(page);break;case "Drive":drive(page);break;case "Learning":learning(page);break;default:settings(page);}
    }

    private void home(LinearLayout p) {
        card(p,"ROBOT STATUS",null);
        LinearLayout row=row(); addLiveMetric(row,"MODE","mode"); addLiveMetric(row,"TOF","tofLive"); p.addView(row);
        row=row();addLiveMetric(row,"VIBRATION","vibrationLive");addLiveMetric(row,"PWM COMMAND","pwmLive");p.addView(row);
        row=row();addLiveMetric(row,"LOCATION","locationLive");addLiveMetric(row,"MAP CONFIDENCE","uncertaintyLive");p.addView(row);
        TextView safety=label(safetyText(),14);safety.setTag("safetyLive");safety.setTextColor(bool("safety",true)?MINT:AMBER);safety.setPadding(dp(3),dp(12),dp(3),dp(12));p.addView(safety);
        Button stop=button("EMERGENCY STOP",RED);p.addView(stop);stop.setOnClickListener(v->sendCmd("stop"));
        Button quick=button("OPEN LIVE MAP",MINT);p.addView(quick);quick.setOnClickListener(v->{tab="Map";buildNav();renderTab();});
        card(p,"LIVE SENSOR HEALTH",null);TextView health=label(healthText(),13);health.setTag("healthLive");health.setTextColor(MUTED);p.addView(health);
    }

    private void map(LinearLayout p) {
        card(p,"TOPOLOGICAL MAP","Junctions and road connections discovered by the robot. Coordinates are relative estimates, not physical centimetres.");
        mapView=new MapView(this);LinearLayout.LayoutParams mp=new LinearLayout.LayoutParams(-1,dp(370));mp.bottomMargin=dp(12);p.addView(mapView,mp);
        TextView legend=label("● Robot   ● Junction   ● Destination\nGreen: smooth/known   Amber: rough   Red: blocked",12);legend.setTextColor(MUTED);p.addView(legend);
        rowAction(p,"Center map","center",()->{if(mapView!=null)mapView.invalidate();});
        TextView stats=label(mapStatsText(),13);stats.setTag("mapStatsLive");stats.setTextColor(MUTED);p.addView(stats);
        card(p,"DESTINATION MARKER", "Place the marker at the intended endpoint. When detected, assign its name here.");
        EditText name=input("Destination name (e.g. Hospital)");p.addView(name);Button assign=button("NAME DETECTED MARKER",MINT);p.addView(assign);assign.setOnClickListener(v->{String n=name.getText().toString().trim();if(n.isEmpty()){toast("Enter a destination name first");return;}sendCmd("assign","name",n);});
    }

    private void drive(LinearLayout p) {
        card(p,"OPERATING MODE","Select a mode before enabling the motors.");
        rowAction(p,"Exploration mode","EXPLORE",()->sendCmd("mode_explore"));rowAction(p,"Destination mode","DESTINATION",()->sendCmd("mode_dest"));rowAction(p,"Manual mode","MANUAL",()->sendCmd("mode_manual"));
        Button enable=button("ENABLE DRIVE",MINT);p.addView(enable);enable.setOnClickListener(v->new AlertDialog.Builder(this).setTitle("Enable robot movement?").setMessage("Place the robot on a clear test area. Keep the emergency stop accessible.").setNegativeButton("Cancel",null).setPositiveButton("Enable",(d,w)->sendCmd("enable")).show());
        Button stop=button("STOP",RED);p.addView(stop);stop.setOnClickListener(v->sendCmd("stop"));
        card(p,"DESTINATION", "Choose a destination that the robot has already learned.");JSONArray ds=array("destinations");for(int i=0;i<ds.length();i++){JSONObject o=ds.optJSONObject(i);if(o!=null){String nm=o.optString("name","Destination");int slot=o.optInt("slot",-1);rowAction(p,nm,"GO",()->sendCmd("go","target",String.valueOf(slot)));}}
        card(p,"MANUAL CONTROL","Hold a direction to move. Releasing the button sends STOP. Use wheels-raised testing first.");
        LinearLayout r=row();r.setGravity(Gravity.CENTER);r.addView(new View(this),new LinearLayout.LayoutParams(dp(72),dp(56)));r.addView(holdButton("▲","manual_forward"),new LinearLayout.LayoutParams(dp(72),dp(56)));r.addView(new View(this),new LinearLayout.LayoutParams(dp(72),dp(56)));p.addView(r);
        r=row();r.setGravity(Gravity.CENTER);r.addView(holdButton("◀","manual_left"),new LinearLayout.LayoutParams(dp(72),dp(56)));Button c=button("STOP",RED);r.addView(c,new LinearLayout.LayoutParams(dp(72),dp(56)));c.setOnClickListener(v->sendCmd("stop"));r.addView(holdButton("▶","manual_right"),new LinearLayout.LayoutParams(dp(72),dp(56)));p.addView(r);
        r=row();r.setGravity(Gravity.CENTER);r.addView(new View(this),new LinearLayout.LayoutParams(dp(72),dp(56)));r.addView(holdButton("▼","manual_reverse"),new LinearLayout.LayoutParams(dp(72),dp(56)));r.addView(new View(this),new LinearLayout.LayoutParams(dp(72),dp(56)));p.addView(r);
    }

    private void learning(LinearLayout p) {
        card(p,"SNN SESSION","Neural learning is held in ESP32 RAM and clears when the board reboots or you erase robot memory.");
        LinearLayout r=row();addLiveMetric(r,"SYNAPSES","synapsesLive");addLiveMetric(r,"CONCEPTS","conceptsLive");p.addView(r);
        r=row();addLiveMetric(r,"PREDICTION ERROR","predictionLive");addLiveMetric(r,"CONFIDENCE","confidenceLive");p.addView(r);
        r=row();addLiveMetric(r,"TRIPS","tripsLive");addLiveMetric(r,"SUCCESS / FAIL","successFailLive");p.addView(r);
        rowAction(p,"Record trip success","+",()->sendCmd("mark_success"));rowAction(p,"Record trip failure","!",()->sendCmd("mark_failure"));
        card(p,"RECENT PHONE REPORTS", "Snapshots are stored locally on this phone. They are separate from the robot's live RAM memory.");TextView reports=label("Saved snapshots: "+reportCount(),13);reports.setTextColor(MUTED);p.addView(reports);
        rowAction(p,"Export reports as JSON","EXPORT",()->exportReports());rowAction(p,"Delete phone reports","DELETE",()->confirmDeleteReports());
    }

    private void settings(LinearLayout p) {
        card(p,"CONNECTION","The app talks directly to the ESP32 access point. No PC, cloud service, or Blynk is used.");
        rowAction(p,"Robot address",BASE,()->toast("Connect to Neuromorphic-Robot Wi-Fi, then use 192.168.4.1"));
        rowAction(p,"Refresh connection","↻",()->fetchState());
        card(p,"MEMORY MANAGEMENT","Erase learned junctions, roads, destinations, neural weights, concept prototypes, and session trip counters on the robot. This cannot be undone.");
        Button erase=button("ERASE ALL ROBOT MEMORY",RED);p.addView(erase);erase.setOnClickListener(v->new AlertDialog.Builder(this).setTitle("Erase robot memory?").setMessage("This clears the current learned map, named destinations, SNN weights and concept memory on the ESP32. The robot will start learning again from an empty session. Phone reports are not deleted by this action.").setNegativeButton("Cancel",null).setPositiveButton("Erase memory",(d,w)->sendCmd("reset_memory")).show());
        card(p,"PHONE DATA","Trip reports are stored separately on this phone. Delete them here independently.");Button clear=button("DELETE ALL PHONE REPORTS",RED);p.addView(clear);clear.setOnClickListener(v->confirmDeleteReports());
        TextView foot=label("App v1.0 · ESP32 direct Wi-Fi · Local-only reports\nEmergency safety remains in robot firmware. Never rely on the app as the only stop mechanism.",12);foot.setTextColor(MUTED);foot.setPadding(0,dp(18),0,dp(8));p.addView(foot);
    }

    private void addLiveMetric(LinearLayout r,String title,String key){LinearLayout box=new LinearLayout(this);box.setOrientation(LinearLayout.VERTICAL);box.setPadding(dp(12),dp(13),dp(8),dp(13));box.setBackground(makeBg(PANEL,14));LinearLayout.LayoutParams lp=new LinearLayout.LayoutParams(0,-2,1);lp.setMargins(dp(3),dp(3),dp(3),dp(3));r.addView(box,lp);TextView a=label(title,10);a.setTextColor(MUTED);a.setTypeface(null,Typeface.BOLD);box.addView(a);TextView b=label(liveValue(key),18);b.setTag(key);b.setTextColor(TEXT);b.setTypeface(null,Typeface.BOLD);b.setPadding(0,dp(5),0,0);box.addView(b);}
    private String liveValue(String key){switch(key){case "mode":return str("mode","—");case "tofLive":return tofText();case "vibrationLive":return fmt(num("vibration",0),3)+" g*";case "pwmLive":return integer("pwm",0)+" / 255";case "locationLive":return integer("currentNode",-1)<0?"Unmapped":"J"+integer("currentNode",-1);case "uncertaintyLive":return str("mapUncertainty","unknown");case "synapsesLive":return String.valueOf(integer("synapses",0));case "conceptsLive":return String.valueOf(integer("concepts",0));case "predictionLive":return fmt(num("predictionError",0),3);case "confidenceLive":return fmt(num("confidence",0),3);case "tripsLive":return String.valueOf(integer("trips",0));case "successFailLive":return integer("successes",0)+" / "+integer("failures",0);default:return "—";}}
    private String safetyText(){return "●  "+(bool("safety",true)?"MOTORS STOPPED / SAFETY ACTIVE":"CHECK SAFETY STATUS");}
    private String healthText(){return "ToF: "+(bool("tofOK",false)?"Connected":"Fault")+"     MPU6050: "+(bool("mpuOK",false)?"Connected":"Fault")+"\nIR mask: "+integer("irMask",0)+"     Closing speed: "+fmt(num("closingSpeed",0),1)+" cm/s\n"+str("message","Waiting for robot…");}
    private String mapStatsText(){return "Locations: "+array("nodes").length()+"    Roads: "+array("edges").length()+"    Destinations: "+array("destinations").length()+"\nMap uncertainty: "+str("mapUncertainty","unknown");}
    private void updateLiveViews(LinearLayout parent){for(int i=0;i<parent.getChildCount();i++){View v=parent.getChildAt(i);Object tag=v.getTag();if(tag instanceof String){String k=(String)tag;if(k.equals("safetyLive")){((TextView)v).setText(safetyText());((TextView)v).setTextColor(bool("safety",true)?MINT:AMBER);}else if(k.equals("healthLive"))((TextView)v).setText(healthText());else if(k.equals("mapStatsLive"))((TextView)v).setText(mapStatsText());else if(k.endsWith("Live")||k.equals("mode"))((TextView)v).setText(liveValue(k));}if(v instanceof LinearLayout)updateLiveViews((LinearLayout)v);else if(v instanceof ScrollView){View c=((ScrollView)v).getChildAt(0);if(c instanceof LinearLayout)updateLiveViews((LinearLayout)c);}}}

    private void startPolling(){polling=true;handler.post(new Runnable(){@Override public void run(){if(!polling)return;fetchState();handler.postDelayed(this,900);}});}
    private void fetchState(){if(fetching)return;fetching=true;net.execute(()->{try{HttpURLConnection c=(HttpURLConnection)new URL(BASE+"/api").openConnection();c.setConnectTimeout(1000);c.setReadTimeout(1200);c.setRequestMethod("GET");StringBuilder b=new StringBuilder();try(BufferedReader r=new BufferedReader(new InputStreamReader(c.getInputStream(),StandardCharsets.UTF_8))){String line;while((line=r.readLine())!=null)b.append(line);}c.disconnect();JSONObject fresh=new JSONObject(b.toString());handler.post(()->{state=fresh;connection.setText("● ONLINE");connection.setTextColor(MINT);if(mapView!=null)mapView.invalidate();updateLiveViews(body);if(System.currentTimeMillis()-lastReportMs>5000){saveReportSnapshot();lastReportMs=System.currentTimeMillis();}});}catch(Exception e){handler.post(()->{connection.setText("● OFFLINE");connection.setTextColor(RED);});}finally{fetching=false;}});}

    private void sendCmd(String cmd){sendCmd(cmd,null,null);}private void sendCmd(String cmd,String key,String value){net.execute(()->{try{String url=BASE+"/cmd?cmd="+enc(cmd);if(key!=null)url+="&"+enc(key)+"="+enc(value);HttpURLConnection c=(HttpURLConnection)new URL(url).openConnection();c.setConnectTimeout(1500);c.setReadTimeout(1500);c.getInputStream().close();c.disconnect();handler.post(()->{toast("Command sent: "+cmd);fetchState();});}catch(Exception e){handler.post(()->toast("Robot not reachable. Connect to its Wi-Fi first."));}});}
    private void saveReportSnapshot(){try{JSONArray a=new JSONArray(prefs.getString("reports","[]"));JSONObject x=new JSONObject();x.put("time",System.currentTimeMillis());x.put("mode",str("mode","unknown"));x.put("node",integer("currentNode",-1));x.put("tof",num("tof",-1));x.put("vibration",num("vibration",0));x.put("predictionError",num("predictionError",0));x.put("confidence",num("confidence",0));x.put("message",str("message",""));a.put(x);while(a.length()>1500)a.remove(0);prefs.edit().putString("reports",a.toString()).apply();}catch(Exception ignored){}}
    private int reportCount(){try{return new JSONArray(prefs.getString("reports","[]")).length();}catch(Exception e){return 0;}}
    private void confirmDeleteReports(){new AlertDialog.Builder(this).setTitle("Delete phone reports?").setMessage("This removes saved report snapshots from this phone. It does not erase robot learning.").setNegativeButton("Cancel",null).setPositiveButton("Delete",(d,w)->{prefs.edit().putString("reports","[]").apply();toast("Phone reports deleted");renderTab();}).show();}
    private void exportReports(){try{String json=prefs.getString("reports","[]");android.content.Intent send=new android.content.Intent(android.content.Intent.ACTION_SEND);send.setType("application/json");send.putExtra(android.content.Intent.EXTRA_TEXT,json);startActivity(android.content.Intent.createChooser(send,"Export robot reports"));}catch(Exception e){toast("Could not export reports");}}

    private class MapView extends View {
        Paint paint=new Paint(3);float downX,downY;JSONObject dragged=null;MapView(Context c){super(c);setBackground(makeBg(Color.rgb(11,18,33),16));setLayerType(View.LAYER_TYPE_SOFTWARE,null);}
        @Override protected void onDraw(Canvas c){super.onDraw(c);JSONArray ns=array("nodes"),es=array("edges");float w=getWidth(),h=getHeight();paint.setColor(Color.rgb(25,38,59));paint.setStrokeWidth(1);for(int x=0;x<w;x+=dp(28))c.drawLine(x,0,x,h,paint);for(int y=0;y<h;y+=dp(28))c.drawLine(0,y,w,y,paint);
            if(ns.length()==0){paint.setColor(MUTED);paint.setTextSize(dp(14));c.drawText("Map appears as the robot explores",dp(20),h/2,paint);return;}
            float minX=0,maxX=1,minY=0,maxY=1;for(int i=0;i<ns.length();i++){JSONObject n=ns.optJSONObject(i);if(n!=null){float x=(float)n.optDouble("x",0),y=(float)n.optDouble("y",0);minX=Math.min(minX,x);maxX=Math.max(maxX,x);minY=Math.min(minY,y);maxY=Math.max(maxY,y);}}
            float sx=(w-dp(64))/Math.max(1,maxX-minX+1),sy=(h-dp(64))/Math.max(1,maxY-minY+1),scale=Math.min(sx,sy);if(scale>dp(70))scale=dp(70);final float sc=scale;java.util.HashMap<Integer,float[]> pts=new java.util.HashMap<>();for(int i=0;i<ns.length();i++){JSONObject n=ns.optJSONObject(i);if(n!=null){int id=n.optInt("id",-1);pts.put(id,new float[]{dp(32)+(float)(n.optDouble("x",0)-minX+.5)*sc,dp(32)+(float)(n.optDouble("y",0)-minY+.5)*sc});}}
            for(int i=0;i<es.length();i++){JSONObject e=es.optJSONObject(i);if(e==null)continue;float[] a=pts.get(e.optInt("a",-1)),b=pts.get(e.optInt("b",-1));if(a==null||b==null)continue;paint.setStyle(Paint.Style.STROKE);paint.setStrokeWidth(dp(3));paint.setColor(e.optBoolean("blocked",false)?RED:"rough".equals(e.optString("roadType"))?AMBER:MINT);if(android.os.Build.VERSION.SDK_INT>=21)paint.setStrokeCap(Paint.Cap.ROUND);c.drawLine(a[0],a[1],b[0],b[1],paint);}
            for(int i=0;i<ns.length();i++){JSONObject n=ns.optJSONObject(i);if(n==null)continue;int id=n.optInt("id",-1);float[] p=pts.get(id);if(p==null)continue;boolean current=id==integer("currentNode",-999),dest=n.optInt("destinationIndex",-1)>=0;paint.setStyle(Paint.Style.FILL);paint.setColor(current?MINT:dest?AMBER:BLUE);c.drawCircle(p[0],p[1],dp(current?9:7),paint);paint.setColor(TEXT);paint.setTextSize(dp(11));paint.setTypeface(Typeface.create("sans-serif-medium",Typeface.BOLD));c.drawText(dest?"D"+id:"J"+id,p[0]+dp(9),p[1]-dp(8),paint);}
        }
        @Override public boolean onTouchEvent(MotionEvent e){if(e.getAction()==MotionEvent.ACTION_DOWN){downX=e.getX();downY=e.getY();return true;}if(e.getAction()==MotionEvent.ACTION_UP){performClick();return true;}return true;}
        @Override public boolean performClick(){super.performClick();return true;}
    }

    private Button holdButton(String text,String cmd){Button b=button(text,MINT);b.setTextSize(23);b.setOnTouchListener((v,e)->{if(e.getAction()==MotionEvent.ACTION_DOWN){sendCmd(cmd);return true;}if(e.getAction()==MotionEvent.ACTION_UP||e.getAction()==MotionEvent.ACTION_CANCEL){sendCmd("manual_stop");return true;}return true;});return b;}
    private void card(LinearLayout p,String title,String subtitle){TextView t=label(title,11);t.setTextColor(MUTED);t.setTypeface(null,Typeface.BOLD);t.setLetterSpacing(.08f);t.setPadding(dp(2),dp(16),dp(2),dp(7));p.addView(t);if(subtitle!=null){TextView s=label(subtitle,12);s.setTextColor(MUTED);s.setPadding(dp(2),0,dp(2),dp(9));p.addView(s);}}
    private LinearLayout row(){LinearLayout r=new LinearLayout(this);r.setOrientation(LinearLayout.HORIZONTAL);r.setPadding(0,dp(3),0,dp(3));return r;}
    private void addMetric(LinearLayout r,String title,String value){LinearLayout box=new LinearLayout(this);box.setOrientation(LinearLayout.VERTICAL);box.setPadding(dp(12),dp(13),dp(8),dp(13));box.setBackground(makeBg(PANEL,14));LinearLayout.LayoutParams lp=new LinearLayout.LayoutParams(0,-2,1);lp.setMargins(dp(3),dp(3),dp(3),dp(3));r.addView(box,lp);TextView a=label(title,10);a.setTextColor(MUTED);a.setTypeface(null,Typeface.BOLD);box.addView(a);TextView b=label(value,18);b.setTextColor(TEXT);b.setTypeface(null,Typeface.BOLD);b.setPadding(0,dp(5),0,0);box.addView(b);}
    private TextView label(String s,int size){TextView t=new TextView(this);t.setText(s);t.setTextColor(TEXT);t.setTextSize(size);t.setGravity(Gravity.CENTER_VERTICAL);return t;}
    private Button button(String text,int color){Button b=new Button(this);b.setText(text);b.setTextColor(color==RED?Color.WHITE:Color.rgb(9,20,32));b.setTextSize(12);b.setTypeface(null,Typeface.BOLD);b.setAllCaps(false);b.setPadding(dp(10),dp(8),dp(10),dp(8));b.setBackground(makeBg(color,14));LinearLayout.LayoutParams lp=new LinearLayout.LayoutParams(-1,-2);lp.setMargins(0,dp(5),0,dp(5));b.setLayoutParams(lp);return b;}
    private EditText input(String hint){EditText e=new EditText(this);e.setSingleLine(true);e.setTextColor(TEXT);e.setHintTextColor(MUTED);e.setHint(hint);e.setTextSize(14);e.setPadding(dp(13),dp(8),dp(13),dp(8));e.setBackground(makeBg(PANEL2,12));LinearLayout.LayoutParams lp=new LinearLayout.LayoutParams(-1,-2);lp.setMargins(0,dp(4),0,dp(8));e.setLayoutParams(lp);return e;}
    private void rowAction(LinearLayout p,String title,String trailing,Runnable action){LinearLayout r=row();r.setGravity(Gravity.CENTER_VERTICAL);r.setPadding(dp(12),dp(12),dp(12),dp(12));r.setBackground(makeBg(PANEL,12));LinearLayout.LayoutParams lp=new LinearLayout.LayoutParams(-1,-2);lp.setMargins(0,dp(4),0,dp(4));r.setLayoutParams(lp);TextView a=label(title,14);a.setLayoutParams(new LinearLayout.LayoutParams(0,-2,1));r.addView(a);TextView b=label(trailing,11);b.setTextColor(MINT);b.setTypeface(null,Typeface.BOLD);r.addView(b);r.setOnClickListener(v->action.run());p.addView(r);}
    private android.graphics.drawable.GradientDrawable makeBg(int color,int radius){android.graphics.drawable.GradientDrawable d=new android.graphics.drawable.GradientDrawable();d.setColor(color);d.setCornerRadius(dp(radius));if(color==PANEL||color==PANEL2){d.setStroke(dp(1),Color.rgb(38,50,72));}return d;}
    private int dp(float v){return (int)(v*getResources().getDisplayMetrics().density+.5f);}private String str(String k,String d){return state.optString(k,d);}private boolean bool(String k,boolean d){return state.optBoolean(k,d);}private double num(String k,double d){return state.optDouble(k,d);}private int integer(String k,int d){return state.has(k)?state.optInt(k,d):d;}private JSONArray array(String k){JSONArray a=state.optJSONArray(k);return a==null?new JSONArray():a;}private String fmt(double n,int d){return String.format(java.util.Locale.US,"%."+d+"f",n);}private String tofText(){return bool("tofValid",false)?fmt(num("tof",0),1)+" cm":"Invalid";}private String enc(String s)throws Exception{return URLEncoder.encode(s,"UTF-8");}private void toast(String s){Toast.makeText(this,s,Toast.LENGTH_SHORT).show();}
    @Override protected void onDestroy(){polling=false;handler.removeCallbacksAndMessages(null);net.shutdownNow();super.onDestroy();}
}
