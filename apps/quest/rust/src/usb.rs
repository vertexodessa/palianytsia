//! UsbManager from Rust (the Quest app has no Java): find the RTL88xx cards,
//! ask for permission once per card, open them and hand out the usbfs fds for
//! `libusb_wrap_sys_device`. Runs on the supervisor thread, attached to
//! the VM for good.
use jni::objects::{GlobalRef, JObject, JString, JValue};
use jni::JNIEnv;

/// devourer-sys kScanPids: 8812EU, 8812AU (881a), 8812AU. A hub's RTL8153
/// Ethernet (0bda:8153) is Realtek too and must not match.
const REALTEK: i32 = 0x0bda;
const CARD_PIDS: [i32; 3] = [0xa81a, 0x881a, 0x8812];
/// Permission requests go out as a broadcast nobody listens to: the grant
/// itself is what matters, and the supervisor polls hasPermission.
const ACTION_USB: &str = "org.palianytsia.quest.USB_PERMISSION";
/// PendingIntent.FLAG_MUTABLE (API 31): UsbManager adds extras to it.
const FLAG_MUTABLE: i32 = 0x0200_0000;

pub struct Usb<'a> {
    act: JObject<'a>,
    mgr: GlobalRef,
}

/// An open card: the connection stays referenced until `close`.
pub struct Open {
    pub name: String,
    pub id: String,
    pub fd: i32,
    conn: GlobalRef,
}

/// An attached card as UsbManager lists it.
pub struct Dev {
    /// usbfs path (`/dev/bus/usb/...`): the key of getDeviceList.
    pub name: String,
    /// vid:pid, for the status line.
    pub id: String,
}

fn jstring(env: &mut JNIEnv, o: JObject) -> jni::errors::Result<String> {
    if o.is_null() {
        return Ok(String::new());
    }
    let js = JString::from(o);
    let s: String = env.get_string(&js)?.into();
    Ok(s)
}

impl<'a> Usb<'a> {
    /// `act`: the Activity jobject (AndroidApp::activity_as_ptr, a global ref).
    pub fn new(env: &mut JNIEnv<'a>, act: JObject<'a>) -> jni::errors::Result<Self> {
        let svc = env.new_string("usb")?;
        let mgr = env
            .call_method(
                &act,
                "getSystemService",
                "(Ljava/lang/String;)Ljava/lang/Object;",
                &[(&svc).into()],
            )?
            .l()?;
        let mgr = env.new_global_ref(mgr)?;
        Ok(Self { act, mgr })
    }

    /// Every attached card, sorted by device name so card 0 stays the same
    /// card from one start to the next.
    pub fn cards(&self, env: &mut JNIEnv) -> jni::errors::Result<Vec<Dev>> {
        let map = self.devices(env)?;
        let vals = env
            .call_method(&map, "values", "()Ljava/util/Collection;", &[])?
            .l()?;
        let arr = jni::objects::JObjectArray::from(
            env.call_method(&vals, "toArray", "()[Ljava/lang/Object;", &[])?
                .l()?,
        );
        let mut out = Vec::new();
        for i in 0..env.get_array_length(&arr)? {
            let d = env.get_object_array_element(&arr, i)?;
            let vid = env.call_method(&d, "getVendorId", "()I", &[])?.i()?;
            let pid = env.call_method(&d, "getProductId", "()I", &[])?.i()?;
            if vid != REALTEK || !CARD_PIDS.contains(&pid) {
                continue;
            }
            let n = env
                .call_method(&d, "getDeviceName", "()Ljava/lang/String;", &[])?
                .l()?;
            out.push(Dev {
                name: jstring(env, n)?,
                id: format!("{vid:04x}:{pid:04x}"),
            });
        }
        out.sort_by(|a, b| a.name.cmp(&b.name));
        Ok(out)
    }

    /// Whether the app may open this card; asks once per device name
    /// (`asked`) when it may not. False also when the card is gone.
    pub fn permitted(
        &self,
        env: &mut JNIEnv,
        name: &str,
        asked: &mut Vec<String>,
    ) -> jni::errors::Result<bool> {
        let Some(dev) = self.device(env, name)? else {
            return Ok(false);
        };
        let has = env
            .call_method(
                &self.mgr,
                "hasPermission",
                "(Landroid/hardware/usb/UsbDevice;)Z",
                &[(&dev).into()],
            )?
            .z()?;
        if !has && !asked.iter().any(|a| a == name) {
            self.request(env, &dev)?;
            asked.push(name.to_string());
        }
        Ok(has)
    }

    /// Opens a permitted card; Err(reason) when it is gone or will not open.
    pub fn open(&self, env: &mut JNIEnv, d: &Dev) -> jni::errors::Result<Result<Open, String>> {
        let Some(dev) = self.device(env, &d.name)? else {
            return Ok(Err(format!("{}: gone", d.id)));
        };
        let conn = env
            .call_method(
                &self.mgr,
                "openDevice",
                "(Landroid/hardware/usb/UsbDevice;)Landroid/hardware/usb/UsbDeviceConnection;",
                &[(&dev).into()],
            )?
            .l()?;
        if conn.is_null() {
            return Ok(Err(format!("{}: openDevice returned null", d.id)));
        }
        let fd = env
            .call_method(&conn, "getFileDescriptor", "()I", &[])?
            .i()?;
        Ok(Ok(Open {
            name: d.name.clone(),
            id: d.id.clone(),
            fd,
            conn: env.new_global_ref(conn)?,
        }))
    }

    /// Whether the device with this name is still on the bus.
    pub fn present(&self, env: &mut JNIEnv, name: &str) -> jni::errors::Result<bool> {
        Ok(self.device(env, name)?.is_some())
    }

    pub fn close(env: &mut JNIEnv, o: Open) {
        if env.call_method(&o.conn, "close", "()V", &[]).is_err() {
            let _ = env.exception_clear();
        }
    }

    fn devices<'b>(&self, env: &mut JNIEnv<'b>) -> jni::errors::Result<JObject<'b>> {
        env.call_method(&self.mgr, "getDeviceList", "()Ljava/util/HashMap;", &[])?
            .l()
    }

    fn device<'b>(
        &self,
        env: &mut JNIEnv<'b>,
        name: &str,
    ) -> jni::errors::Result<Option<JObject<'b>>> {
        let map = self.devices(env)?;
        let key = env.new_string(name)?;
        let d = env
            .call_method(
                &map,
                "get",
                "(Ljava/lang/Object;)Ljava/lang/Object;",
                &[(&key).into()],
            )?
            .l()?;
        Ok((!d.is_null()).then_some(d))
    }

    fn request(&self, env: &mut JNIEnv, dev: &JObject) -> jni::errors::Result<()> {
        let action = env.new_string(ACTION_USB)?;
        let intent = env.new_object(
            "android/content/Intent",
            "(Ljava/lang/String;)V",
            &[JValue::Object(&action)],
        )?;
        let pkg = env
            .call_method(&self.act, "getPackageName", "()Ljava/lang/String;", &[])?
            .l()?;
        env.call_method(
            &intent,
            "setPackage",
            "(Ljava/lang/String;)Landroid/content/Intent;",
            &[JValue::Object(&pkg)],
        )?;
        let pi = env
            .call_static_method(
                "android/app/PendingIntent",
                "getBroadcast",
                "(Landroid/content/Context;ILandroid/content/Intent;I)Landroid/app/PendingIntent;",
                &[
                    JValue::Object(&self.act),
                    JValue::Int(0),
                    JValue::Object(&intent),
                    JValue::Int(FLAG_MUTABLE),
                ],
            )?
            .l()?;
        env.call_method(
            &self.mgr,
            "requestPermission",
            "(Landroid/hardware/usb/UsbDevice;Landroid/app/PendingIntent;)V",
            &[JValue::Object(dev), JValue::Object(&pi)],
        )?;
        log::info!("usb: asked for permission");
        Ok(())
    }
}
