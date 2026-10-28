// MQTT -> Firebase Realtime Database bridge
// Keeps the same DB layout your front end already reads:
//   devices/<id>/live          (latest position + status)
//   devices/<id>/track/<epoch> ({la, lo, s, h})
//   devices/<id>/online        ("online" | "offline")
//
// Setup:
//   npm init -y && npm i mqtt firebase-admin
//   put your Firebase service-account key next to this file as serviceAccount.json
//   MQTT_URL=mqtts://YOUR-CLUSTER.s1.eu.hivemq.cloud:8883 MQTT_USER=bridge MQTT_PASS=xxx node bridge.js
//
// Run it somewhere that stays on (small VPS, Oracle free VM, Raspberry Pi, etc.).

const admin = require("firebase-admin");

const mqtt = require("mqtt");

const { initializeApp, cert } = require("firebase-admin/app");
const { getDatabase, ServerValue } = require("firebase-admin/database");

const serviceAccount = require("./serviceAccount.json");

const app = initializeApp({
  credential: cert(serviceAccount),
  databaseURL: "https://tracker-6c988-default-rtdb.firebaseio.com",
});

const db = getDatabase(app);

const client = mqtt.connect("mqtt://broker.hivemq.com:1883", {
  clientId: "tracker-bridge-" + Math.random().toString(16).slice(2, 10),
  reconnectPeriod: 5000,
  connectTimeout: 15000,
});

client.on("connect", () => {
  console.log("MQTT connected");
  client.subscribe("trk/+/#", { qos: 1 });
});
client.on("error", (e) => {
  console.error("MQTT error details:", e);
  console.error("Error code:", e.code);
  console.error("Error message:", e.message);
});

client.on("message", async (topic, payload) => {
  const [, id, kind] = topic.split("/");
  const text = payload.toString();

  try {
    if (kind === "live") {
      const d = JSON.parse(text);
      d.seen = ServerValue.TIMESTAMP;
      await db.ref(`devices/${id}/live`).set(d);
    } else if (kind === "track") {
      const { p } = JSON.parse(text);
      const upd = {};
      for (const [t, la, lo, s, h] of p) {
        upd[`devices/${id}/track/${t}`] = { la, lo, s, h };
      }
      await db.ref().update(upd);
    } else if (kind === "status") {
      await db.ref(`devices/${id}/online`).set(text);
    }
  } catch (e) {
    console.error("bridge error on", topic, e.message);
  }
});
