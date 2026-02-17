import streamlit as st

st.set_page_config(layout="wide", page_title="Drone Mission Dashboard")

st.title("🚁 Autonomous Drone Mission Control")

st.markdown("""
This dashboard shows real-time drone telemetry, path tracking, safety alerts,
and mission control systems.
""")

st.components.v1.iframe("http://127.0.0.1:5050", height=900, scrolling=True)
