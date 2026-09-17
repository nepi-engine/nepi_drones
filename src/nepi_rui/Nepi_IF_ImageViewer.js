/*
#
# Copyright (c) 2024 Numurus <https://www.numurus.com>.
#
# This file is part of nepi rui (nepi_rui) repo
# (see https://github.com/nepi-engine/nepi_rui)
#
# License: NEPI RUI repo source-code and NEPI Images that use this source-code
# are licensed under the "Numurus Software License", 
# which can be found at: <https://numurus.com/wp-content/uploads/Numurus-Software-License-Terms.pdf>
#
# Redistributions in source code must retain this top-level comment block.
# Plagiarizing this software to sidestep the license obligations is illegal.
#
# Contact Information:
# ====================
# - mailto:nepi@numurus.com
#
 */
import React, { Component } from "react"

//import moment from "moment"
import { observer, inject } from "mobx-react"

//import { TransformWrapper, TransformComponent } from "react-zoom-pan-pinch";

import Section from "./Section"
//import EnableAdjustment from "./EnableAdjustment"
import Button, { ButtonMenu } from "./Button"
import Toggle from "react-toggle"
import AsyncToggle from "./AsyncToggle"
import Label from "./Label"
import RangeAdjustment from "./RangeAdjustment"
import { SliderAdjustment} from "./AdjustmentWidgets"
import { Column, Columns } from "./Columns"
import Styles from "./Styles"
import Input from "./Input"

import NepiIFConfig from "./Nepi_IF_Config"
import NepiIFNavPose from "./Nepi_IF_NavPose"
import NepiIFTransform from "./Nepi_IF_Transform"
import Select, { Option } from "./Select"
import NepiIFSaveData from "./Nepi_IF_SaveData"


import { onChangeChangeStateValue, onChangeSwitchStateValue, createMenuFirstLastName, setElementStyleModified, clearElementStyleModified } from "./Utilities"

function round(value, decimals = 0) {
  return Number(value).toFixed(decimals)
  //return value && Number(Math.round(value + "e" + decimals) + "e-" + decimals)
}


const styles = Styles.Create({

  
  canvas: {
    width: "100%",
    height: "auto",
    transform: "scale(1)"
  }
})

const COMPRESSION_HIGH_QUALITY = 95
// const COMPRESSION_MED_QUALITY = 50
// const COMPRESSION_LOW_QUALITY = 10

const MAX_STREAM_RATE = 20

const PORT = 9091
const ROS_WEBCAM_URL_BASE = `http://${
  window.location.hostname
}:${PORT}/stream?topic=`

@inject("ros")
@observer
class Nepi_IF_ImageViewer extends Component {
  constructor(props) {
    super(props)

    this.state = {
      
      image_topic: 'None',
      prev_image_topic: 'None',
      image_index: 0,
      select_updated_topic: '',
      pixel: null,
      mouse_drag: false,

      save_data_topic: '',
      navpose_topic: '',
      transform_topic: '',
      selected_frame: 'None',

      status_msg: null,


      has_status: false,
      show_config: false,
      show_info: false,
      show_render: false,
      show_overlay: false,
      has_overlay: false,
      has_navpose: false,
      show_navpose: false,
      selected_control: 'NONE',
      hasInitialized: false,
      shouldUpdate: true,
      streamWidth: null,
      streamHeight: null,
      streamSize: 0,
      currentStreamingImageQuality: COMPRESSION_HIGH_QUALITY,
      currentStreamingImageRate: MAX_STREAM_RATE,
      status_listenter: null,
      // Status topic the live listener is actually subscribed to. Compared
      // against the requested topic in updateStatusListener() so a repeat call
      // for the same topic is a no-op -- see the comment there.
      subscribed_status_topic: null,


      click_count: 0,
      crosshair_name: null,

      free_rotate: false,
      rotate_deg_input: '',
      image_width_input: '',
      image_height_input: '',
      swap_box: false,

      connected: false
    }

    
    this.getBaseNamespace = this.getBaseNamespace.bind(this)
    this.updateFrame = this.updateFrame.bind(this)
    this.onCanvasRef = this.onCanvasRef.bind(this)
    this.updateImageSource = this.updateImageSource.bind(this)
    this.getStreamQuality = this.getStreamQuality.bind(this)
    this.onChangeImageQuality = this.onChangeImageQuality.bind(this)
    this.onChangeImageRate = this.onChangeImageRate.bind(this)

    this.createSaveFileName = this.createSaveFileName.bind(this)
    this.saveImageFrame = this.saveImageFrame.bind(this)

    this.renderImageViewer = this.renderImageViewer.bind(this)
    this.renderFilterControls = this.renderFilterControls.bind(this)
    this.renderRenderControls = this.renderRenderControls.bind(this)
    this.renderLiveControls = this.renderLiveControls.bind(this)
    this.renderStreamControls = this.renderStreamControls.bind(this)
    this.applyAspectAll = this.applyAspectAll.bind(this)
    this.renderAspectControls = this.renderAspectControls.bind(this)
    this.renderORControls = this.renderORControls.bind(this)
    this.renderTextControls = this.renderTextControls.bind(this)
    this.renderCrosshairsControls = this.renderCrosshairsControls.bind(this)
    this.renderTargetsControls = this.renderTargetsControls.bind(this)

    this.renderStats = this.renderStats.bind(this)
    this.getImgStatsText = this.getImgStatsText.bind(this)

    this.mouseDownEvent = this.mouseDownEvent.bind(this)
    this.mouseDragEvent = this.mouseDragEvent.bind(this)
    this.mouseUpEvent = this.mouseUpEvent.bind(this)
    this.mouseScrollEvent = this.mouseScrollEvent.bind(this)

    this.sendImageMouseEventMsg = this.sendImageMouseEventMsg.bind(this)

    this.onKeySaveInputOverlayValue = this.onKeySaveInputOverlayValue.bind(this)
    this.onUpdateInputOverlayValue = this.onUpdateInputOverlayValue.bind(this)

    this.onUpdateRotateDegInput = this.onUpdateRotateDegInput.bind(this)
    this.onKeyRotateDegInput = this.onKeyRotateDegInput.bind(this)

    // this.onUpdateImageWidthInput = this.onUpdateImageWidthInput.bind(this)
    // this.onUpdateImageHeightInput = this.onUpdateImageHeightInput.bind(this)
    // this.onKeyImageSizeInput = this.onKeyImageSizeInput.bind(this)

    this.onUpdateAspectWidthInput = this.onUpdateAspectWidthInput.bind(this)
    this.onUpdateAspectHeightInput = this.onUpdateAspectHeightInput.bind(this)
    this.onKeyAspectRatioInput = this.onKeyAspectRatioInput.bind(this)

    //this.ZoomViewer = this.ZoomViewer.bind(this)

    this.statusListener = this.statusListener.bind(this)
    this.updateStatusListener = this.updateStatusListener.bind(this)

    this.onChangeNavposeFrame = this.onChangeNavposeFrame.bind(this)
    this.renderNavPoseSection = this.renderNavPoseSection.bind(this)

  }

  getBaseNamespace(){
    const { namespacePrefix, deviceId} = this.props.ros
    var baseNamespace = null
    if (namespacePrefix !== null && deviceId !== null){
      baseNamespace = "/" + namespacePrefix + "/" + deviceId
    }
    return baseNamespace
  }

  // Callback for handling ROS Status messages
  statusListener(message) {
    this.setState({
      status_msg: message,
      save_data_topic: message.save_data_topic,
      navpose_topic: message.navpose_topic,
      transform_topic: message.transform_topic
    })

  }


  onChangeNavposeFrame(event) {
    const frame = event.target.value
    this.setState({ selected_frame: frame })
    const tf = this.state.transform_topic
    if (tf !== undefined && tf !== '' && tf !== 'None') {
      this.props.ros.sendStringMsg(tf + '/set_source_ref', frame)
    }
  }

  // Renders the NavPose Data section: a reference-frame selector (navpose manager
  // frames), the selected frame's NavPose readout, and the camera mount transform.
  renderNavPoseSection() {
    const { navpose_frames, navpose_frames_topics } = this.props.ros
    const nav_frames = navpose_frames || []
    const nav_frame_topics = navpose_frames_topics || []
    var sel_frame = this.state.selected_frame
    if ((sel_frame === 'None' || sel_frame === '' || nav_frames.indexOf(sel_frame) === -1) && nav_frames.length > 0) {
      sel_frame = nav_frames[0]
    }
    const sel_idx = nav_frames.indexOf(sel_frame)
    var navpose_ns = this.state.navpose_topic
    if ((navpose_ns == null || navpose_ns === '' || navpose_ns === 'None') && sel_idx >= 0 && nav_frame_topics[sel_idx]) {
      navpose_ns = nav_frame_topics[sel_idx] + '/navpose'
    }
    const transform_topic = this.state.transform_topic
    const has_transform = (transform_topic !== undefined && transform_topic !== '' && transform_topic !== 'None')
    // Transformed navpose is published at <idx_ns>/navpose (reference navpose + mount transform)
    var transformed_ns = ''
    const tf_suffix = '/navpose_frame_transform'
    if (has_transform === true && transform_topic.endsWith(tf_suffix)) {
      transformed_ns = transform_topic.slice(0, transform_topic.length - tf_suffix.length) + '/navpose'
    }
    // Always display the camera's navpose (reference navpose + mount transform), published
    // at <idx_ns>/navpose. Fall back to the raw reference-frame navpose only if this device
    // has no mount transform (transformed_ns unavailable).
    const navpose_display_ns = transformed_ns !== '' ? transformed_ns : navpose_ns
    return (
      <React.Fragment>

      <div style={{ borderTop: "1px solid #ffffff", marginTop: Styles.vars.spacing.medium, marginBottom: Styles.vars.spacing.xs }}/>

        {(nav_frames.length > 0) ?
        <Label title={"Reference Frame"}>
          <Select value={sel_frame} onChange={this.onChangeNavposeFrame}>
            {nav_frames.map((f, i) => <Option key={i} value={f}>{f}</Option>)}
          </Select>
        </Label>
        : null }

        <NepiIFNavPose
          navposeNamespace={navpose_display_ns}
          title={"NavPose Data"}
          show_line={false}
          read_only={true}
          make_section={false}
        />

        {(has_transform === true) ?
        <NepiIFTransform
          transformNamespace={transform_topic}
          title={"Camera Frame Transform"}
          device_transform={true}
          show_line={false}
          show_config={false}
          make_section={false}
        />
        : null }

      </React.Fragment>
    )
  }

  // Function for configuring and subscribing to Status
  //
  // Idempotent by design. Re-pointing the listener tears the subscription down
  // and clears status_msg, and componentDidUpdate's guard reads a value derived
  // from status_msg (stream_compression_ratio, which falls back to
  // COMPRESSION_HIGH_QUALITY when status_msg is null) -- so an unconditional
  // re-subscribe here feeds itself. The ImageStatus publisher is latched, so a
  // message lands the instant the new subscription is made, which flips that
  // derived value straight back and re-enters this function. The result is a
  // subscribe/unsubscribe churn at render speed, visible in the rosbridge log as
  // paired "Subscribed to"/"Unsubscribed from" lines many times per second.
  // Bailing out when neither topic has actually changed breaks the cycle no
  // matter which caller re-enters.
  updateStatusListener() {
    // Test for null as well as undefined. Parents commonly derive image_topic
    // from a status message field and hand over an explicit null before the
    // first status arrives -- an undefined-only test stores that null into
    // state.image_topic, where three dozen readers then treat it as a topic
    // string ('None' is the sentinel they all check for, not null).
    const image_topic = (this.props.image_topic !== undefined && this.props.image_topic !== null) ? this.props.image_topic : "None"
    const status_topic = (this.props.status_topic !== undefined && this.props.status_topic !== null) ? this.props.status_topic : image_topic
    const prev_image_topic = (this.state.image_topic != null) ? this.state.image_topic : 'None'
    const image_index = (this.props.image_index !== undefined) ? this.props.image_index : 0

    if (this.state.status_listenter != null &&
        this.state.subscribed_status_topic === status_topic &&
        this.state.image_topic === image_topic) {
      return
    }

    const select_updated_topic = (this.props.select_updated_topic !== undefined) ? this.props.select_updated_topic : null
    if (this.state.status_listenter != null) {
      this.state.status_listenter.unsubscribe()
      this.setState({status_msg: null,
                    status_listenter: null,
                    subscribed_status_topic: null,
                    image_topic: 'None'
      })

    }
    if (status_topic !== 'None' && status_topic !== 'None Available' && status_topic != null ){
      var status_listenter = this.props.ros.setupStatusListener(
            status_topic + '/status',
            "nepi_interfaces/ImageStatus",
            this.statusListener
          )
    }
    this.setState({ status_listenter: status_listenter,
                    subscribed_status_topic: (status_listenter != null) ? status_topic : null,
                    image_topic: image_topic,
                    prev_image_topic: prev_image_topic,
                    image_index: image_index,
                    select_updated_topic: select_updated_topic
    })

    if ((prev_image_topic !== image_topic && prev_image_topic !== 'None' && status_topic !== 'None Available') && select_updated_topic !== null){
      //this.props.ros.sendImageSelectionMsg(select_updated_topic, image_index, image_topic , prev_image_topic)
      this.props.ros.sendStringMsg(select_updated_topic,image_topic)

    }
  }


  updateImageSource() {
    if (this.props.image_topic) {
      if (!this.image) {
        this.image = new Image() // EXPERIMENT -- Only create a new Image when strictly required
      }
      this.image.crossOrigin = "Anonymous"
      this.image.onload = () => {
        const { width, height } = this.image
        this.setState(
          {
            hasInitialized: true,
            connected: false,
            streamWidth: width,
            streamHeight: height,
            streamSize: width * height
          },
          () => {
            requestAnimationFrame(this.updateFrame)
          }
        )
      }
    }
    if (this.image) {
      //const { streamingImageQuality } = this.props.ros
      const stream_quality = this.getStreamQuality()
      const stream_rate = (this.props.streamingImageRate !== undefined) ? this.props.streamingImageRate : MAX_STREAM_RATE
      this.image.src = ROS_WEBCAM_URL_BASE + this.props.image_topic + '&type=mjpeg&framerate=' + stream_rate + '&quality=' + stream_quality
    }
  }

  // JPEG quality percent (1-100) for the web_video_server stream URL.
  //
  // The wire carries stream_compression_ratio, a 0.0-1.0 COMPRESSION level where
  // higher means more compression -- the "Compression Level" slider publishes it
  // as min 0 / max 100 scaled by 0.01, and data_if runs it through check_ratio.
  // Passing that number through as the quality asks web_video_server for quality
  // 0.5, or for 0 outright, since data_if reports a ratio of 0 when compression
  // is disabled. Either clamps to the minimum and yields a macroblocked frame
  // with its chroma stripped. Invert and scale instead, so a ratio of 0.0 (no
  // compression, or compression disabled) means full quality.
  getStreamQuality() {
    if (this.props.streamingImageQuality !== undefined) {
      return this.props.streamingImageQuality
    }
    const status_msg = this.state.status_msg
    const ratio = (status_msg != null) ? status_msg.stream_compression_ratio : null
    if (ratio == null || isNaN(ratio)) {
      return COMPRESSION_HIGH_QUALITY
    }
    const quality = Math.round((1.0 - ratio) * 100)
    return Math.min(100, Math.max(1, quality))
  }

  // Lifecycle method called when the props change.
  // Used to track changes in the image topic value
  componentDidUpdate(prevProps, prevState, snapshot) {
    const { image_topic } = this.props
    const size = this.state.streamSize
    const width = (this.image) ? this.image.width : 0
    const height = (this.image) ? this.image.height : 0
    const got_size = width * height
    const size_changed = (size !== got_size)
    const stream_quality = this.getStreamQuality()
    const stream_rate = (this.props.streamingImageRate !== undefined) ? this.props.streamingImageRate : MAX_STREAM_RATE
    if (prevProps.image_topic !== image_topic || size_changed === true || prevState.currentStreamingImageQuality !== stream_quality){
      this.setState({streamSize: got_size, currentStreamingImageQuality: stream_quality, currentStreamingImageRate: stream_rate})
      this.onChangeImageQuality(stream_quality)
      this.updateImageSource()

    }

    // The status subscription is re-pointed only when the topic it listens to
    // moves. It is deliberately NOT part of the guard above: stream_quality is
    // derived from status_msg, and updateStatusListener() clears status_msg, so
    // driving the listener from a quality change makes the two feed each other.
    if (prevProps.image_topic !== image_topic || prevProps.status_topic !== this.props.status_topic) {
      this.updateStatusListener()
    }

    // Reset the render image-size inputs when the published render size changes
    // externally (new value on the wire, reset, or a different image topic).
    const cur_w = (this.state.status_msg != null) ? this.state.status_msg.width_px : null
    const prev_w = (prevState.status_msg != null) ? prevState.status_msg.width_px : null
    const cur_h = (this.state.status_msg != null) ? this.state.status_msg.height_px : null
    const prev_h = (prevState.status_msg != null) ? prevState.status_msg.height_px : null
    if (cur_w !== prev_w || prevProps.image_topic !== image_topic) {
      this.setState({ image_width_input: (cur_w != null) ? String(cur_w) : '' })
    }
    if (cur_h !== prev_h || prevProps.image_topic !== image_topic) {
      this.setState({ image_height_input: (cur_h != null) ? String(cur_h) : '' })
    }
  }

  componentWillUnmount() {
    this.setState({ shouldUpdate: false })
    if (this.image) {
      this.image.src = null
    }
    // Drop the ImageStatus subscription with the component. Without this the
    // listener outlives every unmount, so navigating away and back leaves one
    // more subscriber on the topic each time.
    if (this.state.status_listenter != null) {
      this.state.status_listenter.unsubscribe()
    }
  }

  componentDidMount() {
    this.updateImageSource()
    this.updateStatusListener()
  }



  onCanvasRef(ref) {
    this.canvas = ref
    if (ref != null){
          this.canvas.addEventListener('mousedown', (e) => {
          this.mouseDownEvent(this.canvas, e)
          })

          this.canvas.addEventListener('mousemove', (e) => {
          this.mouseDragEvent(this.canvas, e)
          })

          this.canvas.addEventListener('mouseup', (e) => {
          this.mouseUpEvent(this.canvas, e)
          })

          this.canvas.addEventListener('wheel', (e) => {
          this.mouseScrollEvent(this.canvas, e)
          }, { passive: false })
      }

    //this.updateImageSource()
  }



  onChangeImageQuality(quality) {
    this.props.ros.onChangeStreamingImageQuality(quality)
    this.setState({currentStreamingImageQuality: quality})
    this.updateImageSource()

  }


  onChangeImageRate(Rate) {
    this.props.ros.onChangeStreamingImageRate(Rate)
    this.setState({currentStreamingImageRate: Rate})
    this.updateImageSource()

  }


  // Builds a download filename matching the NEPI save naming structure produced by
  // system_if.py _createFileName with the default image save settings: timestamp on,
  // milliseconds on, timezone token on, UTC time, empty prefix and suffix.
  // Result form: <datetime token>_<title>.png
  //   datetime token: D<YYYY>-<MM>-<DD>T<HH>-<MM>-<SS>p<mmm>TzUTC
  //   title:          createMenuFirstLastName(image_topic) = <source>-<data_product>
  createSaveFileName() {
    const pad = (value, length) => {
      var str = String(value)
      while (str.length < length) {
        str = '0' + str
      }
      return str
    }
    const now = new Date()
    const date_str = 'D' + pad(now.getUTCFullYear(), 4) + '-' + pad(now.getUTCMonth() + 1, 2) + '-' + pad(now.getUTCDate(), 2)
    const time_str = pad(now.getUTCHours(), 2) + '-' + pad(now.getUTCMinutes(), 2) + '-' + pad(now.getUTCSeconds(), 2)
    const ms_str = 'p' + pad(now.getUTCMilliseconds(), 3)
    const tz_str = 'TzUTC'
    const dt_str = date_str + 'T' + time_str + ms_str + tz_str
    const title = (this.props.title !== undefined && this.props.title != null) ? this.props.title : createMenuFirstLastName(this.state.image_topic)
    return dt_str + '_' + title + '.png'
  }


  // Captures the current canvas frame as a PNG and downloads it to the viewing
  // computer's browser Downloads folder. Does not touch the device save interface.
  saveImageFrame() {
    const image_topic = this.state.image_topic
    if (image_topic == null || image_topic === 'None' || image_topic === 'None Available') {
      return
    }
    const canvas = this.canvas
    if (!canvas || canvas.width === 0 || canvas.height === 0) {
      return
    }
    const filename = this.createSaveFileName()
    canvas.toBlob((blob) => {
      if (blob == null) {
        return
      }
      const url = URL.createObjectURL(blob)
      const link = document.createElement('a')
      link.href = url
      link.download = filename
      document.body.appendChild(link)
      link.click()
      document.body.removeChild(link)
      URL.revokeObjectURL(url)
    }, 'image/png')
  }


  updateFrame() {
    const square_canvas = this.props.squareCanvas ? this.props.squareCanvas : false
    const shouldUpdate = this.state.shouldUpdate
    const hasInitialized = this.state.hasInitialized
    const connected = this.state.connected

    if (shouldUpdate === true && hasInitialized === true && this.canvas) {

      // Firset check for image size change
      const { width, height } = this.image
      const streamWidth = this.state.streamWidth
      const streamHeight = this.state.streamHeight

      const width_changed = width !== streamWidth
      const height_changed = height  !== streamHeight
      const size_changed = (width_changed === true || height_changed === true)
      if (size_changed === true || connected === false) {
        this.setState({
            hasInitialized: true,
            streamWidth: width,
            streamHeight: height,
            streamSize: width * height,
            connected: true
            })

        if (square_canvas === true){
          var size = 0
          if (width > height){
            size = width
          }
          else {
            size = height
          }
          this.canvas.width = size
          this.canvas.height = size

        }
        else {
          this.canvas.width = width
          this.canvas.height = height
        }
      }

      // Then update
      const context = this.canvas.getContext("2d")
      context.fillStyle = "red"
      context.textAlign = "center"
      context.font = "50px Arial"
      context.clearRect(0, 0, streamWidth, streamHeight)
      context.drawImage(this.image, 0, 0, streamWidth, streamHeight)
      //this.setState({ clockTime: moment() })
      requestAnimationFrame(this.updateFrame)
      

    }
  }
  

  getPixelLoc(canvas,event){

      const rect = canvas.getBoundingClientRect()
      const xr = (event.clientX - rect.left) / (canvas.offsetWidth ) 
      const yr = (event.clientY - rect.top) / (canvas.offsetHeight )

      const w = canvas.width
      const h = canvas.height

      const x = Math.floor(w * xr )
      const y = Math.floor(h * yr )

    return [x,y]
  }


  getPixelColor(canvas,x, y){


      // pixelData is a Uint8ClampedArray with 4 values: [R, G, B, A]
      const ctx = canvas.getContext('2d', { willReadFrequently: true })
      const imageData = ctx.getImageData(x, y, 1, 1)
      const pixelData = imageData.data // This is a Uint8ClampedArray [R, G, B, A]
      const r = pixelData[0]
      const g = pixelData[1]
      const b = pixelData[2]
      const a = pixelData[3] / 255 // Alpha is in range [0, 255], convert to [0, 1]
    return [r,g,b,a]
  }

  mouseDownEvent(canvas, event) {
      //const rect = canvas.getBoundingClientRect()
      const [x,y] = this.getPixelLoc(canvas, event)
      const [r,g,b,a] = this.getPixelColor(canvas,x, y)
      this.setState({pixel: [x,y,r,g,b,a], mouse_drag: true})


      //console.log("x coords: " + x + ", y coords: " + y);
  }



  mouseDragEvent(canvas,event){
      // const {sendImageDragMsg} = this.props.ros
      var namespace = (this.props.mouse_event_topic !== undefined && this.props.mouse_event_topic != null) ? this.props.mouse_event_topic : this.state.image_topic + '/mouse_event'
      if (namespace === 'None Available') {
          namespace = 'None'
        }
      const allow_pan_zoom = (this.props.allow_pan_zoom !== undefined) ? this.props.allow_pan_zoom : true
      //const rect = canvas.getBoundingClientRect()

      const is_drag = this.state.mouse_drag
      if (is_drag === true && allow_pan_zoom === true && namespace !== 'None'){
          const drag_namespace = (this.props.mouse_drag_topic !== undefined && this.props.mouse_drag_topic != null) ? this.props.mouse_drag_topic : namespace


          const start_pixel = this.state.pixel
          if (start_pixel !== null){

            const [x1,y1] = [start_pixel[0],start_pixel[1]]
            const [r1,g1,b1,a1] = [start_pixel[2],start_pixel[3],start_pixel[4],start_pixel[5]]
            const mouse_drag_start = {x:x1,y:y1,r:r1,g:g1,b:b1,a:a1}


            const [x2,y2] = this.getPixelLoc(canvas, event)
            const [r2,g2,b2,a2] = this.getPixelColor(canvas,x2, y2)
            const mouse_drag_stop = {x:x2,y:y2,r:r2,g:g2,b:b2,a:a2}


            const dx = Math.abs(x2 - x1) 
            const dy = Math.abs(y2 - y1) 

            const pt = 5
            if (dx > pt || dy > pt){        
              if ( this.state.status_msg != null){
                  this.sendImageMouseEventMsg(drag_namespace,
                                                      this.state.image_topic,
                                                      this.state.image_index,
                                                      null,
                                                      mouse_drag_start, 
                                                      mouse_drag_stop, 
                                                      null, 
                                                      this.state.status_msg
                                                      )
            }
          }
      }
    }
  }

  mouseUpEvent(canvas,event){
      // const {sendImagePixelMsg, sendImageWindowMsg} = this.props.ros
      var namespace = (this.props.mouse_event_topic !== undefined && this.props.mouse_event_topic != null) ? this.props.mouse_event_topic : this.state.image_topic + '/mouse_event'
      if (namespace === 'None Available') {
          namespace = 'None'
        }
      const allow_pan_zoom = (this.props.allow_pan_zoom !== undefined) ? this.props.allow_pan_zoom : true
      //const rect = canvas.getBoundingClientRect()
      if (namespace !== 'None'){
        const [x2,y2] = this.getPixelLoc(canvas, event)
        const pixel = this.state.pixel
        
        if (pixel !== null){
          this.setState({pixel: null, mouse_drag: false})
          const [x1,y1,r,g,b,a] = pixel
          const mouse_click = {x:x1,y:y1,r:r,g:g,b:b,a:a}
          const mouse_window = {x_min:x1, x_max:x2, y_min:y1, y_max:y2}
          const dx = Math.abs(x2 - x1) 
          const dy = Math.abs(y2 - y1) 


          const pt = 5
          if (dx <= pt && dy <= pt){
            var click_namespace = (this.props.mouse_click_topic !== undefined && this.props.mouse_click_topic != null) ? this.props.mouse_click_topic : namespace
            const status_msg = this.state.status_msg
            // const num_crosshairs = status_msg.num_crosshairs
            const click_crosshair_enabled = status_msg.click_crosshair_enabled
            if (click_crosshair_enabled === true && status_msg != null){
              click_namespace = namespace
            }
            // const [r,g,b,a] = this.getPixelColor(canvas,x1, y1)
            //const cur_ms = Date.now()
            //const last_click_ms = this.state.last_click_ms

            // Send Mouse Click Event
            //sendImagePixelMsg(namespace + '/set_click',x1,y1,r,g,b,a)
            const click_count = this.state.click_count + 1
            if (this.state.status_msg != null){
                this.setState({click_count: click_count})
                if (click_count === 1){

                    setTimeout(() => {
                        this.sendImageMouseEventMsg(click_namespace  ,
                                                            this.state.image_topic,
                                                            this.state.image_index,
                                                            mouse_click,
                                                            null, 
                                                            null, 
                                                            null,
                                                            this.state.status_msg
                                                            );
                    }, 500);
                  
                }


            }

            //this.setState({last_click_ms: cur_ms})
          }
          else {
            const window_namespace = (this.props.mouse_window_topic !== undefined && this.props.mouse_window_topic != null) ? this.props.mouse_window_topic : namespace
            const wt = Math.max(canvas.width, canvas.height) * 0.05
            if (dx > wt && dy > wt && allow_pan_zoom === true){
              // Send Mouse Window Event
              //sendImageWindowMsg(namespace + '/set_window',x1,x2,y1,y2)
              if (this.state.status_msg != null){
                  this.sendImageMouseEventMsg(window_namespace  ,
                                                      this.state.image_topic,
                                                      this.state.image_index,
                                                      null, 
                                                      null,
                                                      null,
                                                      mouse_window,
                                                      this.state.status_msg
                                                      )

              }
            }
          }
        }
      }
  }

  mouseScrollEvent(canvas, event){
      var namespace = (this.props.mouse_event_topic !== undefined && this.props.mouse_event_topic != null) ? this.props.mouse_event_topic : this.state.image_topic + '/mouse_event'
      if (namespace === 'None Available') {
          namespace = 'None'
        }
      const allow_pan_zoom = (this.props.allow_pan_zoom !== undefined) ? this.props.allow_pan_zoom : true
      if (allow_pan_zoom !== true || namespace === 'None' || this.state.status_msg == null){
          return
      }
      // Keep the wheel from scrolling the page while zooming the pointcloud
      event.preventDefault()
      const scroll_namespace = (this.props.mouse_scroll_topic !== undefined && this.props.mouse_scroll_topic != null) ? this.props.mouse_scroll_topic : namespace
      const [x,y] = this.getPixelLoc(canvas, event)
      const [r,g,b,a] = this.getPixelColor(canvas, x, y)
      const mouse_scroll = {x:x, y:y, r:r, g:g, b:b, a:a}
      // Normalize the wheel to a unit step: scroll up (deltaY < 0) => zoom in (+1)
      const scroll_amount = (event.deltaY < 0) ? 1.0 : -1.0
      this.sendImageMouseEventMsg(scroll_namespace,
                                          this.state.image_topic,
                                          this.state.image_index,
                                          null,
                                          null,
                                          null,
                                          null,
                                          this.state.status_msg,
                                          mouse_scroll,
                                          scroll_amount
                                          )
  }

  sendImageMouseEventMsg(namespace, image_topic, image_index, mouse_click, mouse_drag_start, mouse_drag_stop , mouse_window, status_msg, mouse_scroll = null, scroll_amount = 0 ) {
    if (mouse_click !== null){
        const status_msg = this.state.status_msg
        // const num_crosshairs = status_msg.num_crosshairs
        // const click_text_enabled = status_msg.click_text_enabled
        // const click_crosshair_enabled = status_msg.click_crosshair_enabled
        // const click_target_enabled = status_msg.click_target_enabled
        // if (click_text_enabled === true && status_msg != null){
        //     const name = 'click'
        //     const width_px = status_msg.width_px
        //     const height_px = status_msg.height_px
        //     const width_deg = status_msg.width_deg
        //     const height_deg = status_msg.height_deg
        //     const x_pixel = mouse_click.x
        //     const y_pixel = mouse_click.y
        //     const x_ratio = x_pixel / (width_px)
        //     const y_ratio = y_pixel / (height_px)
        //     const x_offset_pixel = width_px/2 + (x_ratio - 0.5) * width_px
        //     const y_offset_pixel = height_px/2 + (y_ratio - 0.5) * height_px
        //     const x_offset_deg = width_deg/2 + (x_ratio - 0.5) * width_deg
        //     const y_offset_deg = height_deg/2 + (y_ratio - 0.5) * height_deg
        //     const r = status_msg.crosshairs_color_r
        //     const g = status_msg.crosshairs_color_g
        //     const b = status_msg.crosshairs_color_b
        //     this.props.ros.sendImageCrosshairMsg(namespace.replace('/mouse_event','') + '/add_crosshair_ratios', name, x_pixel, y_pixel, x_ratio, y_ratio, x_offset_deg, y_offset_deg, x_offset_pixel, y_offset_pixel, r, g, b, '' )
        // }
        // if (click_crosshair_enabled === true && status_msg != null){
        //     const name = 'click'
        //     const width_px = status_msg.width_px
        //     const height_px = status_msg.height_px
        //     const width_deg = status_msg.width_deg
        //     const height_deg = status_msg.height_deg
        //     const x_pixel = mouse_click.x
        //     const y_pixel = mouse_click.y
        //     const x_ratio = x_pixel / (width_px)
        //     const y_ratio = y_pixel / (height_px)
        //     const x_offset_pixel = width_px/2 + (x_ratio - 0.5) * width_px
        //     const y_offset_pixel = height_px/2 + (y_ratio - 0.5) * height_px
        //     const x_offset_deg = width_deg/2 + (x_ratio - 0.5) * width_deg
        //     const y_offset_deg = height_deg/2 + (y_ratio - 0.5) * height_deg
        //     const r = status_msg.crosshairs_color_r
        //     const g = status_msg.crosshairs_color_g
        //     const b = status_msg.crosshairs_color_b
        //     this.props.ros.sendImageCrosshairMsg(namespace.replace('/mouse_event','') + '/add_crosshair_ratios', name, x_pixel, y_pixel, x_ratio, y_ratio, x_offset_deg, y_offset_deg, x_offset_pixel, y_offset_pixel, r, g, b, '' )
        // }
        // else if (click_target_enabled === true && status_msg != null) {
        //     const name = 'click'
        //     const width_px = status_msg.width_px
        //     const height_px = status_msg.height_px
        //     const width_deg = status_msg.width_deg
        //     const height_deg = status_msg.height_deg
        //     const x_pixel = mouse_click.x
        //     const y_pixel = mouse_click.y
        //     const x_ratio = x_pixel / (width_px)
        //     const y_ratio = y_pixel / (height_px)
        //     const x_offset_pixel = width_px/2 + (x_ratio - 0.5) * width_px
        //     const y_offset_pixel = height_px/2 + (y_ratio - 0.5) * height_px
        //     const x_offset_deg = width_deg/2 + (x_ratio - 0.5) * width_deg
        //     const y_offset_deg = height_deg/2 + (y_ratio - 0.5) * height_deg
        //     const r = status_msg.crosshairs_color_r
        //     const g = status_msg.crosshairs_color_g
        //     const b = status_msg.crosshairs_color_b
        //     this.props.ros.sendImageTargetMsg(namespace.replace('/mouse_event','') + '/add_target_ratios', name, x_pixel, y_pixel, x_ratio, y_ratio, x_offset_deg, y_offset_deg, x_offset_pixel, y_offset_pixel, r, g, b, '' )
        // }
        
          this.props.ros.sendMouseClickEventMsg(namespace, image_topic, image_index, mouse_click, this.state.click_count, status_msg )
        
        this.setState({click_count: 0})
    }
    else if (mouse_drag_start !== null){
      this.props.ros.sendMouseDragEventMsg(namespace, image_topic, image_index, mouse_drag_start, mouse_drag_stop, status_msg )
    }
    else if (mouse_window !== null){
      this.props.ros.sendMouseWindowEventMsg(namespace, image_topic, image_index, mouse_window, status_msg )
    }
    else if (mouse_scroll !== null){
      this.props.ros.sendMouseScrollEventMsg(namespace, image_topic, image_index, mouse_scroll, scroll_amount, status_msg )
    }

  }



  getImgInfoText(){
    const status_msg = this.state.status_msg
    var msg = ""
    if (status_msg !== null){
      const navpose_frame = status_msg.navpose_frame
      const encoding = status_msg.encoding
      const width_px = round(status_msg.width_px, 0)
      const height_px = round(status_msg.height_px, 0)
      const width_deg = round(status_msg.width_deg, 0)
      const height_deg = round(status_msg.height_deg, 0)
      const perspective = status_msg.perspective

      msg = ("\n\n3D Frame: " + navpose_frame + 
      "\n\nEncoding: " + encoding + 
      "\n\nWidth/Height (Pixals): " + width_px.replace('.','') + ':' + height_px.replace('.','') +
      "\n\nWidth/Height (Deg): " + width_deg.replace('.','') + ':' + height_deg.replace('.','') +
      "\nPerspective: " + perspective )
    }
    else {
      msg = "No Stats Available"

    }
    return msg
  }


  renderInfo() {
   
    if (this.state.status_msg !== null){
      const msg = this.getImgInfoText()
      return (
        <Columns>
        <Column>
         <pre style={{ height: "200px", overflowY: "auto" }} align={"left"} textAlign={"left"}>
                  {msg}
                  </pre>
    
        </Column>
        </Columns>

      )
    }
    else {
      return (
        <Columns>
        <Column>

        </Column>
        </Columns>
      )

    }

  }



  getImgStatsText(){
    const status_msg = this.state.status_msg
    const has_status = this.props.has_status ? this.props.has_status : true
    var msg = ""
    if (status_msg !== null && has_status === true){
      const get_lat = round(status_msg.get_latency_time, 3)
      const pub_lat = round(status_msg.pub_latency_time, 3)
      const proc_time = round(status_msg.process_time, 3)
      const avg_fps = round(status_msg.avg_pub_rate, 3)
      msg = ("\n\nGet Latency: " + get_lat + 
      "\n\nPublish Latency: " + pub_lat + 
      "\n\nProcess Times (Image): " + proc_time +
      "\n\nAvg FPS: " + avg_fps )
    }
    else {
      msg = "No Stats Available"

    }
    return msg
  }

  renderStats() {
    const status_msg = this.state.status_msg
    const has_status = this.props.has_status ? this.props.has_status : true
    if (status_msg != null && has_status === true){
      const msg = this.getImgStatsText()
      return (
        <Columns>
        <Column>
         <pre style={{ height: "200px", overflowY: "auto" }} align={"left"} textAlign={"left"}>
                  {msg}
                  </pre>
    
        </Column>
        </Columns>

      )
    }
    else {
      return (
        <Columns>
        <Column>

        </Column>
        </Columns>
      )

    }

  }

  renderFilters(namespace, filter_options, filter_states, filter_ratios) {
    if (filter_options.length > 0){
      var filter_name = ""
      var filter_enabled = false
      var filter_display_name = 'None'

      var makeFilterClickHandler = (ns, name, enabled) => {
        return () => this.props.ros.sendUpdateBoolMsg(ns + "/set_filter_enable", name, !enabled)
      }

      for (var i = 0; i < filter_options.length; i++) {
        filter_name = filter_options[i]
        filter_enabled = filter_states[i]
        filter_display_name = filter_name.replace('_',' ')
        return (

          <Columns>
          <Column>

                        <Label title={filter_display_name}>

                            <AsyncToggle
                              checked={filter_enabled === true}
                              onClick={makeFilterClickHandler(namespace, filter_name, filter_enabled)}>
                            </AsyncToggle>

                      </Label>

                </Column>
                <Column>

                </Column>
                </Columns>

                )
      }
    }
    else {
      return (
        <Columns>
        <Column>

        </Column>
        </Columns>
      )
    }
  }

  renderFilterControls() {

    const namespace = this.state.image_topic
    const show_filters = this.props.show_filters ? this.props.show_filters : true

    const { imageCaps, sendTriggerMsg, sendBoolMsg } = this.props.ros
    const capabilities = (imageCaps !== null) ? (imageCaps[namespace] !== null ? imageCaps[namespace] : null) : null

   
    if (show_filters === true && this.state.status_msg !== null && namespace !== null && capabilities !== null){
      const has_auto_adjust = (capabilities && capabilities.has_auto_adjust && !this.state.disabled)
      const has_contrast = (capabilities && capabilities.has_contrast && !this.state.disabled)
      const has_brightness = (capabilities && capabilities.has_brightness && !this.state.disabled)
      const has_threshold = (capabilities && capabilities.has_threshold && !this.state.disabled)


      const message = this.state.status_msg
      const auto_adjust_enabled = message.auto_adjust_enabled
      const auto_adjust_ratio = message.auto_adjust_ratio
      const auto_adjust_controls = message.auto_adjust_controls
      const brightness_ratio = message.brightness_ratio
      const contrast_ratio = message.contrast_ratio
      const threshold_ratio = message.threshold_ratio


      const auto_controls = auto_adjust_enabled ? auto_adjust_controls : []
      //Unused const hide_framerate = (!has_framerate || auto_controls.indexOf('framerate') !== -1)
      const hide_brightness = (!has_brightness || auto_controls.indexOf('brightness') !== -1)
      const hide_contrast = (!has_contrast || auto_controls.indexOf('contrast') !== -1)
      const hide_threshold = (!has_threshold || auto_controls.indexOf('threshold') !== -1)



      const filter_options = message.filter_options
      const filter_states = message.filter_states
      const filter_ratios = message.filter_ratios




      return (

        <Columns>
        <Column>
 

            <Label title={"FILTERS"} />
 
        
            {this.renderFilters(namespace, filter_options, filter_states, filter_ratios)}




            <div hidden={(has_auto_adjust !== true )}>
            
                    <Columns>
                    <Column>

                            <Label title={"Auto Adjust"}>
                              <AsyncToggle
                                checked={auto_adjust_enabled===true}
                                onClick={() => sendBoolMsg(namespace + "/set_auto_adjust_enable",!auto_adjust_enabled)}>
                              </AsyncToggle>
                            </Label>


                        </Column>
                        <Column>

                        </Column>
                      </Columns>

                </div>


            <div hidden={(has_auto_adjust !== true || auto_adjust_enabled !== true )}>
                      <SliderAdjustment
                          title={"Auto Adjust Sensitivity"}
                          msgType={"std_msgs/Float32"}
                          adjustment={auto_adjust_ratio}
                          topic={namespace + "/set_auto_adjust_ratio"}
                          scaled={0.01}
                          min={0}
                          max={100}
                          tooltip={"Adjustable Adjust"}
                          noTextBox={true}
                      />

            </div>


          <div hidden={(hide_brightness)}>
                      <SliderAdjustment
                          title={"Brightness"}
                          msgType={"std_msgs/Float32"}
                          adjustment={brightness_ratio}
                          topic={namespace + "/set_brightness_ratio"}
                          scaled={0.01}
                          min={0}
                          max={100}
                          tooltip={"Adjustable brightness"}
                          noTextBox={true}
                      />

            </div>


            <div hidden={(hide_contrast)}>
                      <SliderAdjustment
                        title={"Contrast"}
                        msgType={"std_msgs/Float32"}
                        adjustment={contrast_ratio}
                        topic={namespace + "/set_contrast_ratio"}
                        scaled={0.01}
                        min={0}
                        max={100}
                        tooltip={"Adjustable contrast"}
                        noTextBox={true}
                      />

            </div>

            <div hidden={(hide_threshold)}>
                      <SliderAdjustment
                          title={"Thresholding"}
                          msgType={"std_msgs/Float32"}
                          adjustment={threshold_ratio}
                          topic={namespace + "/set_threshold_ratio"}
                          scaled={0.01}
                          min={0}
                          max={100}
                          tooltip={"Adjustable threshold"}
                          noTextBox={true}
                      />
              </div>




          <ButtonMenu>
            <Button onClick={() => sendTriggerMsg( namespace + "/reset_filters")}>{"Reset Controls"}</Button>
          </ButtonMenu>


      </Column>
      </Columns>
      )
    }
    else {
      return (
        <Columns>
        <Column>

        </Column>
        </Columns>
      )

    }

  }


  renderRenderControls() {

    const namespace = this.state.image_topic
    const show_render_controls = this.props.show_render_controls ? this.props.show_render_controls : true

    const { imageCaps, sendTriggerMsg } = this.props.ros
    const capabilities = (imageCaps !== null) ? (imageCaps[namespace] !== null ? imageCaps[namespace] : null) : null


    if (show_render_controls === true && this.state.status_msg !== null && namespace !== null && capabilities !== null){
      const has_range = (capabilities && capabilities.has_range && !this.state.disabled)
      const has_zoom = (capabilities && capabilities.has_zoom && !this.state.disabled)
      const has_pan = (capabilities && capabilities.has_pan && !this.state.disabled)
      const has_window = (capabilities && capabilities.has_window && !this.state.disabled)


      const message = this.state.status_msg
      const auto_adjust_enabled = message.auto_adjust_enabled
      const auto_adjust_controls = message.auto_adjust_controls
      const range_start_ratio = message.range_ratios.start_range
      const range_stop_ratio = message.range_ratios.stop_range
      const zoom_ratio = message.zoom_ratio
      const pan_x_ratio = message.pan_x_ratio
      const pan_y_ratio = message.pan_y_ratio
      const x_min_ratio = message.window_x_ratios.start_range
      const x_max_ratio = message.window_x_ratios.stop_range
      const y_min_ratio = message.window_y_ratios.start_range
      const y_max_ratio = message.window_y_ratios.stop_range

      // const live_adjustments_disabled = message.live_adjustments_disabled
      // const live_adjust_enabled = message.live_adjust_enabled
      // const live_adjust_rotate_ratio = message.live_adjust_rotate_ratio
      // const live_adjust_rotate_deg = message.live_adjust_rotate_deg
      // const live_adjust_x_ratio = message.live_adjust_x_ratio
      // const live_adjust_x_pixels = message.live_adjust_x_pixels
      // const live_adjust_x_deg = message.live_adjust_x_degs
      // const live_adjust_y_ratio = message.live_adjust_y_ratio
      // const live_adjust_y_pixels = message.live_adjust_y_pixels
      // const live_adjust_y_deg = message.live_adjust_y_degs

      const auto_controls = auto_adjust_enabled ? auto_adjust_controls : []
      const hide_range = (!has_range || auto_controls.indexOf('range') !== -1)
      const hide_window = (!has_window || auto_controls.indexOf('window') !== -1)

      const min_range_m_adj = round( message.min_range_m_adj,1)
      const max_range_m_adj = round(message.max_range_m_adj,1)

      // The render image-size control applies only to the pointcloud render
      // (fewer pixels = faster render). For 2D image products width_px/height_px
      // are the source dimensions, so hide it there.
      // const is_pointcloud = (message.data_source_description !== undefined
          // && message.data_source_description !== null
          // && String(message.data_source_description).toLowerCase().indexOf('pointcloud') !== -1)

      return (

        <Columns>
        <Column>


            <Label title={"RENDER  CONTROLS"} />

          {/* <div hidden={(is_pointcloud !== true)}>
            <Columns>
            <Column>
                <Label title={"Render Width (px)"}>
                <Input
                  id={"render_image_width"}
                  value={this.state.image_width_input}
                  onChange={this.onUpdateImageWidthInput}
                  onKeyDown={this.onKeyImageSizeInput}
                  style={{ width: "80%" }}
                />
                </Label>
            </Column>
            <Column>
                <Label title={"Render Height (px)"}>
                <Input
                  id={"render_image_height"}
                  value={this.state.image_height_input}
                  onChange={this.onUpdateImageHeightInput}
                  onKeyDown={this.onKeyImageSizeInput}
                  style={{ width: "80%" }}
                />
                </Label>
            </Column>
            </Columns>
          </div> */}
 
    

            <div hidden={(hide_range)}>
                    <RangeAdjustment
                      title="Range Clip"
                      min={range_start_ratio}
                      max={range_stop_ratio}
                      min_limit_m={0.2}
                      max_limit_m={1.0}
                      topic={namespace + "/set_range_ratios"}
                      tooltip={"Adjustable range ratio"}
                      noTextBox={true}
                    />
          </div>


          <Columns>
          <Column>

              <div hidden={hide_range}>
                <Label title={"Min Range (m)"}>
                <Input
                  value={min_range_m_adj}
                  disabled={true}
                  style={{ width: "80%" }}
                />
                </Label>

            </div>


              </Column>
              <Column>

              <div hidden={hide_range}>
                <Label title={"Max Range (m)"}>
                <Input
                  value={max_range_m_adj}
                 disabled={true}
                  style={{ width: "80%" }}
                />
                </Label>

            </div>


              </Column>
            </Columns>  



          <div hidden={(has_zoom !== true )}>

                      <SliderAdjustment
                            title={"Zoom"}
                            msgType={"std_msgs/Float32"}
                            adjustment={zoom_ratio}
                            topic={namespace + "/set_zoom_ratio"}
                            scaled={0.01}
                            min={0}
                            max={100}
                            disabled={false}
                            tooltip={"Zoom controls"}
                            noTextBox={true}
                        />
          </div>


          <div hidden={(has_pan !== true )}>

                      <SliderAdjustment
                            title={"Pan X"}
                            msgType={"std_msgs/Float32"}
                            adjustment={pan_x_ratio}
                            topic={namespace + "/set_pan_x_ratio"}
                            scaled={0.01}
                            min={0}
                            max={100}
                            disabled={false}
                            tooltip={"Pan left-right controls"}
                            noTextBox={true}
                        />

                  

          </div>


          <div hidden={(hide_window)}>
                    <RangeAdjustment
                      title="X Window"
                      min={x_min_ratio}
                      max={x_max_ratio}
                      min_limit_m={0.2}
                      max_limit_m={1.0}
                      disabled={true}
                      tooltip={"Adjustable x_min and x_max ratios"}
                      noTextBox={true}
                    />

                 
          </div>

          <div hidden={(has_pan !== true )}>



                      <SliderAdjustment
                            title={"Pan Y"}
                            msgType={"std_msgs/Float32"}
                            adjustment={pan_y_ratio}
                            topic={namespace + "/set_pan_y_ratio"}
                            scaled={0.01}
                            min={0}
                            max={100}
                            disabled={false}
                            tooltip={"Pan up-down controls"}
                            noTextBox={true}
                        />

          </div>


          <div hidden={(hide_window)}>


                    <RangeAdjustment
                      title="Y Window"
                      min={y_min_ratio}
                      max={y_max_ratio}
                      min_limit_m={0.2}
                      max_limit_m={1.0}
                      disabled={true}
                      tooltip={"Adjustable y_min and y_max ratios"}
                      noTextBox={true}
                    />
          </div>

          {/* The 3-D Zoom, Rotate and Tilt sliders live in LIVE ADUSTMENTS (see
              renderLiveControls) alongside Translate X/Y, since all of them are
              interactive view adjustments rather than render-output settings.
              This section keeps only what defines the render itself: output
              size and range clipping. */}


                        <ButtonMenu>
                            <Button onClick={() => sendTriggerMsg( namespace + "/reset_renders")}>{"Reset Controls"}</Button>
                          </ButtonMenu>


      </Column>
      </Columns>
      )
    }
    else {
      return (
        <Columns>
        <Column>

        </Column>
        </Columns>
      )

    }

  }



  renderLiveControls() {

    const namespace = this.state.image_topic
    const show_render_controls = this.props.show_render_controls ? this.props.show_render_controls : true

    const { imageCaps, sendTriggerMsg, sendFloatMsg } = this.props.ros
    const capabilities = (imageCaps !== null) ? (imageCaps[namespace] !== null ? imageCaps[namespace] : null) : null


    if (show_render_controls === true && this.state.status_msg !== null && namespace !== null && capabilities !== null){
      // const has_range = (capabilities && capabilities.has_range && !this.state.disabled)
      // const has_zoom = (capabilities && capabilities.has_zoom && !this.state.disabled)
      // const has_pan = (capabilities && capabilities.has_pan && !this.state.disabled)
      // const has_window = (capabilities && capabilities.has_window && !this.state.disabled)
      const has_zoom_3d = (capabilities && capabilities.has_zoom_3d && !this.state.disabled)
      const has_rotate_3d = (capabilities && capabilities.has_rotate_3d && !this.state.disabled)
      const has_tilt_3d = (capabilities && capabilities.has_tilt_3d && !this.state.disabled)


      const message = this.state.status_msg
      // const auto_adjust_enabled = message.auto_adjust_enabled
      // const auto_adjust_controls = message.auto_adjust_controls
      // const range_start_ratio = message.range_ratios.start_range
      // const range_stop_ratio = message.range_ratios.stop_range
      // const zoom_ratio = message.zoom_ratio
      // const pan_x_ratio = message.pan_x_ratio
      // const pan_y_ratio = message.pan_y_ratio
      // const x_min_ratio = message.window_x_ratios.start_range
      // const x_max_ratio = message.window_x_ratios.stop_range
      // const y_min_ratio = message.window_y_ratios.start_range
      // const y_max_ratio = message.window_y_ratios.stop_range
      const zoom_3d_ratio = message.zoom_3d_ratio
      const rotate_3d_ratio = message.rotate_3d_ratio
      const tilt_3d_ratio = message.tilt_3d_ratio

      const live_adjustments_disabled = message.live_adjustments_disabled
      // const live_adjust_enabled = message.live_adjust_enabled
      const live_adjust_rotate_ratio = message.live_adjust_rotate_ratio
      // const live_adjust_rotate_deg = message.live_adjust_rotate_deg
      const live_adjust_x_ratio = message.live_adjust_x_ratio
      // const live_adjust_x_pixels = message.live_adjust_x_pixels
      // const live_adjust_x_deg = message.live_adjust_x_degs
      const live_adjust_y_ratio = message.live_adjust_y_ratio
      // const live_adjust_y_pixels = message.live_adjust_y_pixels
      // const live_adjust_y_deg = message.live_adjust_y_degs

      // const auto_controls = auto_adjust_enabled ? auto_adjust_controls : []
      // const hide_range = (!has_range || auto_controls.indexOf('range') !== -1)
      // const hide_window = (!has_window || auto_controls.indexOf('window') !== -1)

      // const min_range_m_adj = round( message.min_range_m_adj,1)
      // const max_range_m_adj = round(message.max_range_m_adj,1)

      // The render image-size control applies only to the pointcloud render
      // (fewer pixels = faster render). For 2D image products width_px/height_px
      // are the source dimensions, so hide it there.
      // const is_pointcloud = (message.data_source_description !== undefined
          // && message.data_source_description !== null
          // && String(message.data_source_description).toLowerCase().indexOf('pointcloud') !== -1)

      return (

        <Columns>
        <Column>

          <div hidden={live_adjustments_disabled === true}>
            <Label title={"LIVE ADUSTMENTS"} />

                      {/* Zoom 3D / Rotate 3D / Tilt 3D move the render camera around
                          the pointcloud. They are titled "3D" to separate them from the
                          "Rotate" slider below, which rotates the flat output image
                          via live_adjust_rotate_ratio -- two different controls that
                          both used to read simply "Rotate", in two different panels. */}
                      <div hidden={(has_zoom_3d !== true )}>
                      <SliderAdjustment
                            title={"Zoom 3D"}
                            msgType={"std_msgs/Float32"}
                            adjustment={zoom_3d_ratio}
                            topic={namespace + "/set_zoom_3d_ratio"}
                            scaled={0.01}
                            min={0}
                            max={100}
                            disabled={false}
                            tooltip={"Zoom the 3D render camera in and out from the pointcloud"}
                            noTextBox={true}
                        />
                      </div>

                      <div hidden={(has_rotate_3d !== true )}>
                      <SliderAdjustment
                            title={"Rotate 3D"}
                            msgType={"std_msgs/Float32"}
                            adjustment={rotate_3d_ratio}
                            topic={namespace + "/set_rotate_3d_ratio"}
                            scaled={0.01}
                            min={0}
                            max={100}
                            disabled={false}
                            tooltip={"Rotate the 3D render camera around the pointcloud"}
                            noTextBox={true}
                        />
                      </div>

                      <div hidden={(has_tilt_3d !== true )}>
                      <SliderAdjustment
                            title={"Tilt 3D"}
                            msgType={"std_msgs/Float32"}
                            adjustment={tilt_3d_ratio}
                            topic={namespace + "/set_tilt_3d_ratio"}
                            scaled={0.01}
                            min={0}
                            max={100}
                            disabled={false}
                            tooltip={"Tilt the 3D render camera around the pointcloud"}
                            noTextBox={true}
                        />
                      </div>

                      <SliderAdjustment
                            title={"Rotate"}
                            msgType={"std_msgs/Float32"}
                            adjustment={live_adjust_rotate_ratio}
                            topic={namespace + "/set_live_adjust_rotate_ratio"}
                            scaled={0.01}
                            min={0}
                            max={100}
                            disabled={false}
                            tooltip={"Live Rotate control (rotates the output image)"}
                            noTextBox={true}
                        />
                      <SliderAdjustment
                            title={"Translate X"}
                            msgType={"std_msgs/Float32"}
                            adjustment={live_adjust_x_ratio}
                            topic={namespace + "/set_live_adjust_x_ratio"}
                            scaled={0.01}
                            min={0}
                            max={100}
                            disabled={false}
                            tooltip={"Live Translate X control"}
                            noTextBox={true}
                        />
                      <SliderAdjustment
                            title={"Translate Y"}
                            msgType={"std_msgs/Float32"}
                            adjustment={live_adjust_y_ratio}
                            topic={namespace + "/set_live_adjust_y_ratio"}
                            scaled={0.01}
                            min={0}
                            max={100}
                            disabled={false}
                            tooltip={"Live Translate Y control"}
                            noTextBox={true}
                        />


                        {/* Reset Orientation resets the 3-D camera orientation
                            (Rotate 3D, Tilt 3D and zoom) through the backend.
                            Reset Position returns Translate X and Translate Y to
                            their 0.5 neutral -- the same value reset_renders uses
                            for live_adjust_x_ratio / live_adjust_y_ratio. These
                            replace a single Reset Controls button that fired
                            reset_renders and reset far more than this section owns. */}
                        <ButtonMenu>
                            <Button onClick={() => sendTriggerMsg( namespace + "/reset_render_3d_controls")}>{"Reset Orientation"}</Button>
                            <Button onClick={() => {
                              sendFloatMsg( namespace + "/set_live_adjust_x_ratio", 0.5)
                              sendFloatMsg( namespace + "/set_live_adjust_y_ratio", 0.5)
                            }}>{"Reset Position"}</Button>
                          </ButtonMenu>

                </div>

      </Column>
      </Columns>
      )
    }
    else {
      return (
        <Columns>
        <Column>

        </Column>
        </Columns>
      )

    }

  }


  renderStreamControls() {

    const namespace = this.state.image_topic
    const show_render_controls = this.props.show_render_controls ? this.props.show_render_controls : true

    const {
      imageCaps,
      // sendTriggerMsg,
      sendBoolMsg } = this.props.ros
    const capabilities = (imageCaps !== null) ? (imageCaps[namespace] !== null ? imageCaps[namespace] : null) : null


    if (show_render_controls === true && this.state.status_msg !== null && namespace !== null && capabilities !== null){
      
      const message = this.state.status_msg
      const stream_compression_enabled = message.stream_compression_enabled
      const stream_compression_ratio = message.stream_compression_ratio
     
      return (

        <Columns>
        <Column>

            <Label title={"STREAM COMPRESSION"} />


                    <Columns>
                    <Column>

                            <Label title={"Enabled"}>
                              <AsyncToggle
                                checked={stream_compression_enabled===true}
                                onClick={() => sendBoolMsg(namespace + "/set_stream_compression_enable",!stream_compression_enabled)}>
                              </AsyncToggle>
                            </Label>


                        </Column>
                        <Column>

                        </Column>
                      </Columns>

                      <SliderAdjustment
                            title={"Compression Level"}
                            msgType={"std_msgs/Float32"}
                            adjustment={stream_compression_ratio}
                            topic={namespace + "/set_stream_compression_ratio"}
                            scaled={0.01}
                            min={0}
                            max={100}
                            disabled={false}
                            tooltip={"Steam Compression Level control"}
                            noTextBox={true}
                        />
                   


      </Column>
      </Columns>
      )
    }
    else {
      return (
        <Columns>
        <Column>

        </Column>
        </Columns>
      )

    }

  }

  renderORControls() {

    const namespace = this.state.image_topic
    const show_res_orient = this.props.show_res_orient ? this.props.show_res_orient : true

    const { imageCaps, sendTriggerMsg, sendBoolMsg } = this.props.ros
    const capabilities = (imageCaps !== null) ? (imageCaps[namespace] !== null ? imageCaps[namespace] : null) : null

   
    if (show_res_orient === true && this.state.status_msg !== null && namespace !== null && capabilities !== null){
      const has_rotate_2d = (capabilities && capabilities.has_rotate_2d && !this.state.disabled)
      const has_flip_horz = (capabilities && capabilities.has_flip_horz && !this.state.disabled)
      const has_flip_vert = (capabilities && capabilities.has_flip_vert && !this.state.disabled)
      const has_resolution = (capabilities && capabilities.has_resolution && !this.state.disabled)


      const message = this.state.status_msg
      const auto_adjust_enabled = message.auto_adjust_enabled
      const auto_adjust_controls = message.auto_adjust_controls
      const resolution_ratio = message.resolution_ratio
      const resolution_str = message.resolution_current
      const rotate_2d_deg = message.rotate_2d_deg
      const flip_horz = message.flip_horz
      const flip_vert = message.flip_vert

      const auto_controls = auto_adjust_enabled ? auto_adjust_controls : []
      const hide_resolution = (!has_resolution || auto_controls.indexOf('resolution') !== -1)
      const hide_rotate_2d = (!has_rotate_2d || auto_controls.indexOf('rotate_2d_deg') !== -1)
      const hide_flip_horz = (!has_flip_horz || auto_controls.indexOf('flip_horz') !== -1)
      const hide_flip_vert = (!has_flip_vert || auto_controls.indexOf('flip_vert') !== -1)


      return (

        <Columns>
        <Column>
 

        <Label title={"ORIENTATION & SIZE"} />

              <div hidden={(hide_rotate_2d)}>




                          <Columns>
                          <Column>


                            {this.state.free_rotate === false &&
                              <ButtonMenu>
                                <Button onClick={() => sendTriggerMsg( namespace + "/rotate_2d")}>{"Rotate 90 Degs"}</Button>
                              </ButtonMenu>
                            }

                            {this.state.free_rotate === true &&
                              <div>
                                <Label title={"Set Angle (0-359)"}>
                                  <Input
                                    id="free_cam_rotate_deg"
                                    value={this.state.rotate_deg_input}
                                    style={{ width: "100%" }}
                                    onChange={this.onUpdateRotateDegInput}
                                    onKeyDown={this.onKeyRotateDegInput}
                                  />
                                </Label>
                                <Label title={"Swap Box W/H"}>
                                  {/* react-toggle (not AsyncToggle): checked is local view state, already immediate -- no backend round trip to confirm. */}
                                  <Toggle
                                    checked={this.state.swap_box}
                                    onClick={() => {
                                      const new_swap = !this.state.swap_box
                                      this.setState({ swap_box: new_swap })
                                      sendBoolMsg(namespace + '/set_rotate_2d_swap_box', new_swap)
                                    }}
                                  />
                                </Label>
                              </div>

                            }

                          <Label title={"Free Rotate"}>
                              {/* react-toggle (not AsyncToggle): checked is local view state, already immediate -- no backend round trip to confirm. */}
                              <Toggle
                                checked={this.state.free_rotate}
                                onClick={() => this.setState({ free_rotate: !this.state.free_rotate })}
                              />
                            </Label>

                          </Column>
                          <Column>

                          <Label title={"Angle"}>
                        <Input
                          value={rotate_2d_deg}
                          id="cur_rotate_deg"
                          style={{ width: "100%" }}
                          disabled={true}
                        />
                      </Label>

                          </Column>
                        </Columns>

               </div>



              

                          <Columns>
                          <Column>

                          <div hidden={(hide_flip_horz)}>

  
                            <Label title={"Flip Horz"}>
                                <AsyncToggle
                                  checked={flip_horz}
                                  onClick={() => sendBoolMsg(namespace + '/set_flip_horz',!flip_horz)}
                                /> 
                              </Label>

                          </div>

                          </Column>
                          <Column>

                           <div hidden={(hide_flip_vert)}>

  
                            <Label title={"Flip Vert"}>
                                <AsyncToggle
                                  checked={flip_vert}
                                  onClick={() => sendBoolMsg(namespace + '/set_flip_vert',!flip_vert)}
                                /> 
                              </Label>

                          </div>

                          </Column>
                        </Columns>         
 

 
              <div hidden={(hide_resolution)}>



                            <SliderAdjustment
                                  title={"Resolution"}
                                  msgType={"std_msgs/Float32"}
                                  adjustment={resolution_ratio}
                                  topic={namespace + '/set_resolution_ratio'}
                                  scaled={0.01}
                                  min={0}
                                  max={100}
                                  tooltip={"Adjustable Resolution"}
                                  noTextBox={true}
                              />

              </div>

                          <Columns>
                          <Column>

                          </Column>
                          <Column>

                          <Label title={"Current Resolution"}>
                        <Input
                          value={resolution_str}
                          id="cur_res"
                          style={{ width: "100%" }}
                          disabled={true}
                        />
                      </Label>

                          </Column>
                        </Columns>             


          <ButtonMenu>
            <Button onClick={() => sendTriggerMsg( namespace + "/reset_settings")}>{"Reset Controls"}</Button>
          </ButtonMenu>


      </Column>
      </Columns>
      )
    }
    else {
      return (
        <Columns>
        <Column>

        </Column>
        </Columns>
      )

    }

  }



  applyAspectAll(aspect_ratio) {
    const all_namespace = this.getBaseNamespace() + '/all/images'
    const {sendBoolMsg,  sendFloatMsg} = this.props.ros
    
    if (this.state.status_msg !== null && all_namespace !== null){
      sendBoolMsg(all_namespace + '/set_aspect_adjust_enable',true)
      sendFloatMsg(all_namespace + '/set_aspect_adjust_ratio',aspect_ratio)
    }

  }


  onUpdateAspectWidthInput(event) {
    const el = document.getElementById("render_aspect_width")
    setElementStyleModified(el)
    this.setState({ aspect_width_input: event.target.value })
  }

  onUpdateAspectHeightInput(event) {
    const el = document.getElementById("render_aspect_height")
    setElementStyleModified(el)
    this.setState({ aspect_height_input: event.target.value })
  }

  onKeyAspectRatioInput(event) {
    const { sendFloatMsg } = this.props.ros
    if (event.key === 'Enter') {
      const w_el = document.getElementById("render_aspect_width")
      const h_el = document.getElementById("render_aspect_height")
      let width = parseInt(this.state.aspect_width_input, 10)
      let height = parseInt(this.state.aspect_height_input, 10)
      if (isNaN(width) || isNaN(height) || width <= 1 || width >= 100 || height <= 1 || height >= 20) {
        return
      }
      clearElementStyleModified(w_el)
      clearElementStyleModified(h_el)
      const namespace = this.state.image_topic
      const ratio =  width/height
      sendFloatMsg(namespace + '/set_aspect_adjust_ratio',ratio)
      this.setState({ aspect_width_input: String(width), aspect_height_input: String(height) })
    }
  }



  renderAspectControls() {
    const namespace = this.state.image_topic
    // const show_res_orient = this.props.show_res_orient ? this.props.show_res_orient : true

    const {
      // imageCaps,
      // sendTriggerMsg,
      sendBoolMsg } = this.props.ros
    // const capabilities = (imageCaps !== null) ? (imageCaps[namespace] !== null ? imageCaps[namespace] : null) : null

    // const show_overlay_controls = this.props.show_overlay_controls ? this.props.show_overlay_controls : true
   
    if (this.state.status_msg !== null && namespace !== null){
      const message = this.state.status_msg

      const aspect_adjustment_disabled = message.aspect_adjustment_disabled
      const aspect_adjust_enabled = message.aspect_adjust_enabled
      const aspect_ratio_set = round(message.aspect_ratio_set, 2)
      const aspect_by_ratio = aspect_ratio_set - 1
      const aspect_ratio_str = message.aspect_ratio_str
      // const aspect_ratio = message.aspect_ratio
    

      
      return (

        <Columns>
        <Column>

          <div hidden={aspect_adjustment_disabled === true}>
            <Label title={"ASPECT RATIO"} />

            <Columns>
              <Column>
                  
                      <Label title={"Enable"}>
                      <AsyncToggle
                        checked={aspect_adjust_enabled}
                        onClick={() => sendBoolMsg(namespace + '/set_aspect_adjust_enable',!aspect_adjust_enabled)}
                      /> 
                    </Label>


                </Column>
                <Column>


                </Column>
              </Columns>

              <div hidden={aspect_adjust_enabled === false}>

              <Columns>
              <Column>
                  


                <Label title={"Aspect Ratio"}>
                <Input
                  disabled={true}
                  value={aspect_ratio_str}
                  style={{ width: "50%" }}
                />
                </Label>

                </Column>
                <Column>

               
                    <ButtonMenu>
                    <Button onClick={() => this.applyAspectAll(aspect_ratio_set)}>{"Apply All"}</Button>
                  </ButtonMenu>
                  

                </Column>
              </Columns>


                      <SliderAdjustment
                            title={"Apect Ratio"}
                            msgType={"std_msgs/Float32"}
                            adjustment={aspect_by_ratio}
                            topic={namespace + "/set_aspect_adjust_by_ratio"}
                            scaled={0.01}
                            min={0}
                            max={100}
                            disabled={false}
                            tooltip={"Aspect Ratio control"}
                            displayValue={aspect_ratio_set}
                        />

                  <Columns>
                    <Column>
                        <Label title={"Set Width"}>
                        <Input
                          id={"render_aspect_width"}
                          value={this.state.aspect_width_input}
                          onChange={this.onUpdateAspectWidthInput}
                          onKeyDown={this.onKeyAspectRatioInput}
                          style={{ width: "80%" }}
                        />
                        </Label>
                    </Column>
                    <Column>
                        <Label title={"Set Height"}>
                        <Input
                          id={"render_aspect_height"}
                          value={this.state.aspect_height_input}
                          onChange={this.onUpdateAspectHeightInput}
                          onKeyDown={this.onKeyAspectRatioInput}
                          style={{ width: "80%" }}
                        />
                        </Label>
                    </Column>
                    </Columns>




                </div>
          </div>
      </Column>
      </Columns>
      )
    }
    else {
      return (
        <Columns>
        <Column>

        </Column>
        </Columns>
      )

    }

  }

  onUpdateInputOverlayValue(event) {
    this.setState({ custom_overlay_input: event.target.value })
    document.getElementById("input_overlay").style.color = Styles.vars.colors.red
    this.render()
  }

  onKeySaveInputOverlayValue(event) {
    const {sendStringMsg}  = this.props.ros
    if(event.key === 'Enter'){
      const value = this.state.custom_overlay_input
      const namespace = this.state.image_topic
      sendStringMsg(namespace + '/add_overlay_text', value)
      this.setState({custom_overlay_input: ''})
    }
  }

  onUpdateRotateDegInput(event) {
    const el = document.getElementById("free_cam_rotate_deg")
    setElementStyleModified(el)
    this.setState({ rotate_deg_input: event.target.value })
  }

  onKeyRotateDegInput(event) {
    const { sendIntMsg } = this.props.ros
    if (event.key === 'Enter') {
      const el = document.getElementById("free_cam_rotate_deg")
      clearElementStyleModified(el)
      const namespace = this.state.image_topic
      let deg = parseInt(this.state.rotate_deg_input, 10)
      if (isNaN(deg)) {
        this.setState({ rotate_deg_input: '' })
        return
      }
      // Constrain to an integer and normalize to 0-359 (backend also normalizes)
      deg = ((deg % 360) + 360) % 360
      sendIntMsg(namespace + '/set_rotate_2d_deg', deg)
      this.setState({ rotate_deg_input: String(deg) })
    }
  }

  onUpdateImageWidthInput(event) {
    const el = document.getElementById("render_image_width")
    setElementStyleModified(el)
    this.setState({ image_width_input: event.target.value })
  }

  onUpdateImageHeightInput(event) {
    const el = document.getElementById("render_image_height")
    setElementStyleModified(el)
    this.setState({ image_height_input: event.target.value })
  }

  // Enter in either the width or height box sends both dimensions in one
  // ImageSize message to the pointcloud render's set_image_size topic. The
  // backend clamps to 100 < size < 5000, so ignore out-of-range entries here.
  onKeyImageSizeInput(event) {
    const { sendImageSizeMsg } = this.props.ros
    if (event.key === 'Enter') {
      const w_el = document.getElementById("render_image_width")
      const h_el = document.getElementById("render_image_height")
      let width = parseInt(this.state.image_width_input, 10)
      let height = parseInt(this.state.image_height_input, 10)
      if (isNaN(width) || isNaN(height) || width <= 100 || width >= 5000 || height <= 100 || height >= 5000) {
        return
      }
      clearElementStyleModified(w_el)
      clearElementStyleModified(h_el)
      const namespace = this.state.image_topic
      sendImageSizeMsg(namespace + '/set_image_size', width, height)
      this.setState({ image_width_input: String(width), image_height_input: String(height) })
    }
  }

  renderTextControls() {
    const { sendTriggerMsg, sendBoolMsg } = this.props.ros
    const namespace = this.state.image_topic
    
   
    if (this.state.status_msg !== null && namespace !== null){
      const message = this.state.status_msg
      const overlay_text_enabled = message.overlay_text_enabled
      const click_text_enabled = message.click_text_enabled
      const overlay_text_size_ratio = message.overlay_text_size_ratio
      const overlay_text_horz_ratio = message.overlay_text_horz_ratio
      const overlay_text_vert_ratio = message.overlay_text_vert_ratio
      // const overlay_text_transparency_ratio = message.overlay_text_transparency_ratio
      // const overlay_text_color_r = message.overlay_text_color_r
      // const overlay_text_color_g = message.overlay_text_color_g
      // const overlay_text_color_b = message.overlay_text_color_b
      const overlay_text_source_name = message.overlay_text_source_name
      const overlay_text_date_time = message.overlay_text_date_time
      const overlay_text_nav = message.overlay_text_nav
      const overlay_text_pose = message.overlay_text_pose
      

      return (

        <Columns>
        <Column>

      <Label title={"TEXT OVERLAYS"} />
      

            <Columns>
            <Column>
                
                      <Label title={"Show Text"}>
                      <AsyncToggle
                        checked={overlay_text_enabled}
                        onClick={() => sendBoolMsg(namespace + '/set_overlay_text_enable',!overlay_text_enabled)}
                      /> 
                    </Label>

                </Column>
                <Column>

                </Column>
              </Columns>

              <div hidden={overlay_text_enabled === false}>

            <Columns>
            <Column>



                  <SliderAdjustment
                      title={"Text Size"}
                      msgType={"std_msgs/Float32"}
                      adjustment={overlay_text_size_ratio}
                      topic={namespace + "/set_overlay_text_size_ratio"}
                      scaled={0.01}
                      min={0}
                      max={100}
                      disabled={false}
                      tooltip={"Overlay text size controls"}
                      noTextBox={true}
                  />

                  <SliderAdjustment
                      title={"Text Vert Position"}
                      msgType={"std_msgs/Float32"}
                      adjustment={overlay_text_vert_ratio}
                      topic={namespace + "/set_overlay_text_vert_ratio"}
                      scaled={0.01}
                      min={0}
                      max={100}
                      disabled={false}
                      tooltip={"Overlay text vert position controls"}
                      noTextBox={true}
                  />

                  <SliderAdjustment
                      title={"Text Horz Position"}
                      msgType={"std_msgs/Float32"}
                      adjustment={overlay_text_horz_ratio}
                      topic={namespace + "/set_overlay_text_horz_ratio"}
                      scaled={0.01}
                      min={0}
                      max={100}
                      disabled={false}
                      tooltip={"Overlay text horz position controls"}
                      noTextBox={true}
                  />

                        <Label title={"Enable Click Text Position"}>
                              <AsyncToggle
                                checked={click_text_enabled}
                                onClick={() => sendBoolMsg(namespace + '/click_text_enable',!click_text_enabled)}
                              /> 
                            </Label>


            

                      <Label title={'Add'}>
                        <Input id="input_overlay" 
                          value={this.state.custom_overlay_input} 
                          onChange={this.onUpdateInputOverlayValue} 
                          onKeyDown= {this.onKeySaveInputOverlayValue} />
                      </Label>


                    <ButtonMenu>
                    <Button onClick={() => sendTriggerMsg( namespace + "/clear_overlay_text_list")}>{"Clear"}</Button>
                  </ButtonMenu>



                </Column>
                <Column>



                  <Label title={"Source Name"}>
                      <AsyncToggle
                        checked={overlay_text_source_name}
                        onClick={() => sendBoolMsg(namespace + '/set_overlay_text_source_name',!overlay_text_source_name)}
                      /> 
                    </Label>

                    <Label title={"Date Time"}>
                      <AsyncToggle
                        checked={overlay_text_date_time}
                        onClick={() => sendBoolMsg(namespace + '/set_overlay_text_date_time',!overlay_text_date_time)}
                      /> 
                    </Label>

                    <Label title={"Location"}>
                      <AsyncToggle
                        checked={overlay_text_nav}
                        onClick={() => sendBoolMsg(namespace + '/set_overlay_text_nav',!overlay_text_nav)}
                      /> 
                    </Label>

                    <Label title={"Pose"}>
                      <AsyncToggle
                        checked={overlay_text_pose}
                        onClick={() => sendBoolMsg(namespace + '/set_overlay_text_pose',!overlay_text_pose)}
                      /> 
                    </Label>

                </Column>
              </Columns>

                <Columns>
                <Column>
                                    
                  <ButtonMenu>
                    <Button onClick={() => sendTriggerMsg( namespace + "/reset_overlays")}>{"Reset Overlays"}</Button>
                  </ButtonMenu>
                </Column>
                <Column>

                </Column>
              </Columns>



          </div>

      </Column>
      </Columns>
      )
    }
    else {
      return (
        <Columns>
        <Column>

        </Column>
        </Columns>
      )

    }

  }

  renderCrosshairsControls() {
    const { sendTriggerMsg, sendBoolMsg } = this.props.ros
    const namespace = this.state.image_topic
    // const show_overlay_controls = this.props.show_overlay_controls ? this.props.show_overlay_controls : true
   
    if (this.state.status_msg !== null && namespace !== null){
      const message = this.state.status_msg

      const crosshairs_enabled = message.crosshairs_enabled
      const click_crosshair = message.click_crosshair_enabled && crosshairs_enabled === true
      const crosshairs_size_ratio = message.crosshairs_size_ratio
      const crosshairs_thickness_ratio = message.crosshairs_thickness_ratio
      const crosshairs_text_ratio = message.crosshairs_text_ratio
      const crosshair_names = message.overlay_crosshair_names && crosshairs_enabled === true
      const crosshair_pixels = message.overlay_crosshair_pixels && crosshairs_enabled === true
      const crosshair_degrees = message.overlay_crosshair_degrees && crosshairs_enabled === true
      const crosshair_msgs = message.overlay_crosshair_messages && crosshairs_enabled === true
      
      // const crosshairs_color_r = message.crosshairs_color_r
      // const crosshairs_color_g = message.crosshairs_color_g
      // const crosshairs_color_b = message.crosshairs_color_b

      

      return (

        <Columns>
        <Column>

      <Label title={"CROSSHAIR OVERLAYS"} />


            <Columns>
            <Column>
                
                      <Label title={"Show Crosshairs"}>
                      <AsyncToggle
                        checked={crosshairs_enabled}
                        onClick={() => sendBoolMsg(namespace + '/crosshairs_enable',!crosshairs_enabled)}
                      /> 
                    </Label>

                </Column>
                <Column>

                </Column>
              </Columns>

              <div hidden={crosshairs_enabled === false}>

                <Columns>
                <Column>

                       <SliderAdjustment
                            title={"Crosshair Size"}
                            msgType={"std_msgs/Float32"}
                            adjustment={crosshairs_size_ratio}
                            topic={namespace + "/set_crosshairs_size_ratio"}
                            scaled={0.01}
                            min={0}
                            max={100}
                            disabled={false}
                            tooltip={"Overlay crosshair size controls"}
                            noTextBox={true}
                        />

                       <SliderAdjustment
                            title={"Crosshair Thickness"}
                            msgType={"std_msgs/Float32"}
                            adjustment={crosshairs_thickness_ratio}
                            topic={namespace + "/set_crosshairs_thickness_ratio"}
                            scaled={0.01}
                            min={0}
                            max={100}
                            disabled={false}
                            tooltip={"Overlay crosshair thickness controls"}
                            noTextBox={true}
                        />

                       <SliderAdjustment
                            title={"Crosshair Text"}
                            msgType={"std_msgs/Float32"}
                            adjustment={crosshairs_text_ratio}
                            topic={namespace + "/set_crosshairs_text_ratio"}
                            scaled={0.01}
                            min={0}
                            max={100}
                            disabled={false}
                            tooltip={"Overlay crosshair text controls"}
                            noTextBox={true}
                        />
                        
                        <Label title={"Enable Click Crosshair"}>
                              <AsyncToggle
                                disabled={message.zoom_ratio > 0.01}
                                checked={click_crosshair}
                                onClick={() => sendBoolMsg(namespace + '/click_crosshair_enable',!click_crosshair)}
                              /> 
                            </Label>





                </Column>
                <Column>

                            <Label title={"   Show Names"}>
                              <AsyncToggle
                                disabled={crosshairs_enabled === false}
                                checked={crosshair_names}
                                onClick={() => sendBoolMsg(namespace + '/overlay_crosshair_names',!crosshair_names)}
                              /> 
                            </Label>

                          <Label title={"   Show Pixels"}>
                              <AsyncToggle
                                disabled={crosshairs_enabled === false}
                                checked={crosshair_pixels}
                                onClick={() => sendBoolMsg(namespace + '/overlay_crosshair_pixels',!crosshair_pixels)}
                              /> 
                            </Label>

                            <Label title={"   Show Degs"}>
                              <AsyncToggle
                                disabled={crosshairs_enabled === false}
                                checked={crosshair_degrees}
                                onClick={() => sendBoolMsg(namespace + '/overlay_crosshair_degrees',!crosshair_degrees)}
                              /> 
                            </Label>

        
                            <Label title={"   Show Msgs"}>
                              <AsyncToggle
                                disabled={crosshairs_enabled === false}
                                checked={crosshair_msgs}
                                onClick={() => sendBoolMsg(namespace + '/overlay_crosshair_messages',!crosshair_msgs)}
                              /> 
                            </Label>

                </Column>
              </Columns>




                <Columns>
                <Column>
                                    
                        <ButtonMenu>
                          <Button onClick={() => sendTriggerMsg( namespace + "/clear_crosshairs")}>{"Clear Crosshairs"}</Button>
                        </ButtonMenu>  
                </Column>
                <Column>

                </Column>
              </Columns>
                
               </div>

                

      </Column>
      </Columns>
      )
    }
    else {
      return (
        <Columns>
        <Column>

        </Column>
        </Columns>
      )

    }

  }

  renderTargetsControls() {
    const { sendTriggerMsg, sendBoolMsg } = this.props.ros
    const namespace = this.state.image_topic
    // const show_overlay_controls = this.props.show_overlay_controls ? this.props.show_overlay_controls : true
   
    if (this.state.status_msg !== null && namespace !== null){
      const message = this.state.status_msg

      const targets_enabled = message.targets_enabled
      const click_target = message.click_target_enabled && targets_enabled === true
      const targets_size_ratio = message.targets_size_ratio
      const targets_thickness_ratio = message.targets_thickness_ratio
      const targets_text_ratio = message.targets_text_ratio
      const target_names = message.overlay_target_names && targets_enabled === true
      const target_pixels = message.overlay_target_pixels && targets_enabled === true
      const target_degrees = message.overlay_target_degrees && targets_enabled === true
      const target_msgs = message.overlay_target_messages && targets_enabled === true
      
      // const targets_color_r = message.targets_color_r
      // const targets_color_g = message.targets_color_g
      // const targets_color_b = message.targets_color_b

      

      return (

        <Columns>
        <Column>

      <Label title={"TARGET OVERLAYS"} />


            <Columns>
            <Column>
                
                      <Label title={"Show Targets"}>
                      <AsyncToggle
                        checked={targets_enabled}
                        onClick={() => sendBoolMsg(namespace + '/targets_enable',!targets_enabled)}
                      /> 
                    </Label>

                </Column>
                <Column>

                </Column>
              </Columns>

              <div hidden={targets_enabled === false}>

                <Columns>
                <Column>

                       <SliderAdjustment
                            title={"Target Size"}
                            msgType={"std_msgs/Float32"}
                            adjustment={targets_size_ratio}
                            topic={namespace + "/set_targets_size_ratio"}
                            scaled={0.01}
                            min={0}
                            max={100}
                            disabled={false}
                            tooltip={"Overlay target size controls"}
                            noTextBox={true}
                        />

                       <SliderAdjustment
                            title={"Target Thickness"}
                            msgType={"std_msgs/Float32"}
                            adjustment={targets_thickness_ratio}
                            topic={namespace + "/set_targets_thickness_ratio"}
                            scaled={0.01}
                            min={0}
                            max={100}
                            disabled={false}
                            tooltip={"Overlay target thickness controls"}
                            noTextBox={true}
                        />

                       <SliderAdjustment
                            title={"Target Text"}
                            msgType={"std_msgs/Float32"}
                            adjustment={targets_text_ratio}
                            topic={namespace + "/set_targets_text_ratio"}
                            scaled={0.01}
                            min={0}
                            max={100}
                            disabled={false}
                            tooltip={"Overlay target text controls"}
                            noTextBox={true}
                        />
                        
                        <Label title={"Enable Click Target"}>
                              <AsyncToggle
                                disabled={message.zoom_ratio > 0.01}
                                checked={click_target}
                                onClick={() => sendBoolMsg(namespace + '/click_target_enable',!click_target)}
                              /> 
                            </Label>





                </Column>
                <Column>

                            <Label title={"   Show Names"}>
                              <AsyncToggle
                                disabled={targets_enabled === false}
                                checked={target_names}
                                onClick={() => sendBoolMsg(namespace + '/overlay_target_names',!target_names)}
                              /> 
                            </Label>

                          <Label title={"   Show Pixels"}>
                              <AsyncToggle
                                disabled={targets_enabled === false}
                                checked={target_pixels}
                                onClick={() => sendBoolMsg(namespace + '/overlay_target_pixels',!target_pixels)}
                              /> 
                            </Label>

                            <Label title={"   Show Degs"}>
                              <AsyncToggle
                                disabled={targets_enabled === false}
                                checked={target_degrees}
                                onClick={() => sendBoolMsg(namespace + '/overlay_target_degrees',!target_degrees)}
                              /> 
                            </Label>

        
                            <Label title={"   Show Msgs"}>
                              <AsyncToggle
                                disabled={targets_enabled === false}
                                checked={target_msgs}
                                onClick={() => sendBoolMsg(namespace + '/overlay_target_messages',!target_msgs)}
                              /> 
                            </Label>

                </Column>
              </Columns>




                            <Columns>
                <Column>
                                    
                      <ButtonMenu>
                          <Button onClick={() => sendTriggerMsg( namespace + "/clear_targets")}>{"Clear Targets"}</Button>
                        </ButtonMenu>   
                </Column>
                <Column>

                </Column>
              </Columns>
                
              </div>


                

      </Column>
      </Columns>
      )
    }
    else {
      return (
        <Columns>
        <Column>

        </Column>
        </Columns>
      )

    }

  }

  // renderCompression(){
  //   const currentStreamingImageQuality = this.state.currentStreamingImageQuality
  //   const hideQualitySelector = this.props.hideQualitySelector ? this.props.hideQualitySelector: false
  //   const streamingImageQuality = this.props.streamingImageQuality ? 
  //               (this.props.streamingImageQuality != null) ? this.props.streamingImageQuality : this.state.currentStreamingImageQuality
  //               : this.state.currentStreamingImageQuality
    
  //   if (currentStreamingImageQuality !== streamingImageQuality)
  //   {
  //     this.setState({currentStreamingImageQuality: streamingImageQuality})
  //   }

  //   return(
  //         <Columns>
  //         <Column>
  //         <div align={"left"} textAlign={"left"}>
  //           { hideQualitySelector ?
  //             null :
  //             <Label title={"Compression Level"} />
  //           }
  //         </div>
  //         </Column>
  //         <Column>
  //         <div align={"left"} textAlign={"left"}>
  //           { hideQualitySelector ?
  //             null :
  //             <div>
  //               <Label title={"Low"} />
  //               <Toggle
  //                 checked={streamingImageQuality >= COMPRESSION_HIGH_QUALITY}
  //                 onClick={() => {this.onChangeImageQuality(COMPRESSION_HIGH_QUALITY)}}
  //               />
  //             </div>
  //           }
  //         </div>
  //         </Column>
  //         <Column>
  //         <div align={"left"} textAlign={"left"}>
  //           { hideQualitySelector ?
  //             null :
  //             <div>
  //               <Label title={"Medium"} />
  //               <Toggle
  //                 checked={streamingImageQuality >= COMPRESSION_MED_QUALITY && streamingImageQuality < COMPRESSION_HIGH_QUALITY}
  //                 onClick={() => {this.onChangeImageQuality(COMPRESSION_MED_QUALITY)}}
  //               />
  //             </div>
  //           }
  //         </div>
  //         </Column>
  //         <Column>
  //         <div align={"left"} textAlign={"left"}>
  //           { hideQualitySelector ?
  //             null :
  //             <div>
  //               <Label title={"High"} />
  //               <Toggle
  //                 checked={streamingImageQuality <= COMPRESSION_LOW_QUALITY}
  //                 onClick={() => {this.onChangeImageQuality(COMPRESSION_LOW_QUALITY)}}
  //               />
  //             </div>
  //           }
  //         </div>
  //         </Column>
  //       </Columns>

  //   )

  // }






  renderImageViewer() {

    var namespace = this.props.image_topic ? this.props.image_topic : 'None'
    if (namespace === 'None Available') {
        namespace = 'None'
      }
    const { sendTriggerMsg, sendBoolMsg } = this.props.ros

    // 3D render controls (pointcloud viewers only): shown in the header next to the name
    // only when the selected data product is a pointcloud. Prefer the explicit data_product
    // prop (the selected Data Product) when provided; otherwise fall back to the live
    // ImageStatus.data_source_description ('pointcloud' for the pointcloud image node).
    const render_ns = this.state.image_topic
    const status_msg = this.state.status_msg
    const config_topic = (status_msg !== null && status_msg !== undefined) ? status_msg.config_topic : ''
    const data_product = (this.props.data_product !== undefined && this.props.data_product !== null) ? String(this.props.data_product).toLowerCase() : null
    const is_pointcloud = (data_product !== null)
        ? (data_product.indexOf('pointcloud') !== -1)
        : (status_msg !== null && status_msg !== undefined
            && status_msg.data_source_description !== undefined
            && String(status_msg.data_source_description).toLowerCase().indexOf('pointcloud') !== -1)
    const show_3d_controls = (is_pointcloud === true && this.state.status_msg !== null && render_ns !== 'None' && render_ns !== null)
    const render_3d_controls_enabled = (status_msg !== null && status_msg !== undefined && status_msg.render_3d_controls_enabled !== undefined) ? status_msg.render_3d_controls_enabled : false

    const show_save_controls = (this.props.show_save_controls !== undefined) ? this.props.show_save_controls : true
    // const show_all_config_options = (this.props.show_all_config_options !== undefined) ? this.props.show_all_config_options : true
    const show_topic_selector = (this.props.show_topic_selector !== undefined) ? this.props.show_topic_selector : true
    const save_data_topic = (this.props.save_data_topic !== undefined) ? this.props.save_data_topic :  this.state.save_data_topic


    const title = (this.props.title !== undefined && this.props.title != null) ? this.props.title : createMenuFirstLastName(this.state.image_topic)


    const single_slider_topic = (this.props.single_slider_topic !== undefined) ? this.props.single_slider_topic : null
    const single_slider_value = (this.props.single_slider_value !== undefined) ? this.props.single_slider_value : 0.5
    const double_slider_topic = (this.props.double_slider_topic !== undefined) ? this.props.double_slider_topic : null
    const double_slider_values = (this.props.double_slider_values !== undefined) ? (this.props.double_slider_values.length === 2) ? this.props.double_slider_values : [0,0,1.0] : [0,0,1.0]
    
    const { userRestricted} = this.props.ros
    const admin_controls_restricted = userRestricted.indexOf('DATA-IMAGE-ALL-VIEW') !== -1 && userRestricted.indexOf('DATA-IMAGE-ALL-CONTROL') !== -1
    var show_image_controls = (this.props.show_image_controls !== undefined)? this.props.show_image_controls : true
    show_image_controls = (show_image_controls === true && admin_controls_restricted === false)

    const show_info_controls = (this.props.show_info_controls !== undefined) ? this.props.show_info_controls && show_image_controls: show_image_controls
    const show_config_controls = (this.props.show_config_controls !== undefined) ? this.props.show_config_controls && show_image_controls: show_image_controls
    const show_navpose_controls = (this.props.show_navpose_controls !== undefined) ? this.props.show_navpose_controls && show_image_controls: show_image_controls
    const show_render_controls = (this.props.show_render_controls !== undefined) ? this.props.show_render_controls && show_image_controls: show_image_controls
    const show_overlay_controls = (this.props.show_overlay_controls !== undefined) ? this.props.show_overlay_controls && show_image_controls: show_image_controls
    const show_overlay_text = (this.props.show_overlay_text !== undefined) ? this.props.show_overlay_text  && show_overlay_controls === true: true
    const show_overlay_crosshairs = (this.props.show_overlay_crosshairs !== undefined) ? this.props.show_overlay_crosshairs && show_overlay_controls === true : true
    const show_overlay_targets = (this.props.show_overlay_targets !== undefined) ? this.props.show_overlay_targets   && show_overlay_controls === true : true

    const selected_control = this.state.selected_control
    const show_info = selected_control === 'INFO' && (userRestricted.indexOf('DATA-IMAGE-INFO-VIEW') === -1)
    const show_config = selected_control === 'CONFIG' && (userRestricted.indexOf('DATA-IMAGE-CONFIG-VIEW') === -1)
    const show_navpose = selected_control === 'NAVPOSE'  && (userRestricted.indexOf('DATA-IMAGE-NAVPOSE-VIEW') === -1)
    const show_render = selected_control === 'RENDER'  && (userRestricted.indexOf('DATA-IMAGE-RENDER-VIEW') === -1)
    const show_overlay = selected_control === 'OVERLAY'  && (userRestricted.indexOf('DATA-IMAGE-OVERLAY-VIEW') === -1)

    return (
      
      <Columns>
      <Column>

                  
                  
                  <div style={{ display: 'flex', alignItems: 'center' }}>

                        <div style={{ align: 'left' }}>
                          <Label title={title} />
                        </div>


                        {(show_3d_controls === true) ?
                        <div style={{ display: 'flex', alignItems: 'center', marginLeft: Styles.vars.spacing.regular }}>

                                <Label title={"3D Controls"}>
                                  <AsyncToggle
                                    checked={render_3d_controls_enabled===true}
                                    onClick={() => sendBoolMsg(render_ns + "/render_3d_controls",!render_3d_controls_enabled)}>
                                  </AsyncToggle>
                                </Label>

                                {(render_3d_controls_enabled === true) ?
                                <ButtonMenu>
                                  <Button onClick={() => sendTriggerMsg( render_ns + "/reset_render_3d_controls")}>{"Reset Orientation"}</Button>
                                </ButtonMenu>
                                : null }

                                {(render_3d_controls_enabled === true) ?
                                <ButtonMenu>
                                  <Button onClick={() => sendTriggerMsg( render_ns + "/reset_render_3d_position")}>{"Reset Position"}</Button>
                                </ButtonMenu>
                                : null }

                        </div>
                        : null }


{/*
                        Reset and Snapshot buttons removed from here (requested
                        live 2026-09-16: "get rid of the top reset and snapshot
                        buttons" -- there were two Snapshot buttons per viewer,
                        this one and NepiIFSaveData's own further down, and the
                        second "doesn't work"). Snapshot's browser-download
                        logic (saveImageFrame) now runs from NepiIFSaveData's
                        own Snapshot button instead, via the onSnapshotOverride
                        prop passed to it further down.
*/}

                  </div>




                  <canvas style={styles.canvas} ref={this.onCanvasRef} />


                          <div hidden={single_slider_topic === null}>
                              <SliderAdjustment
                                title={"Single Image Slider"}
                                msgType={"std_msgs/Float32"}
                                adjustment={single_slider_value}
                                topic={single_slider_topic}
                                scaled={0.01}
                                min={0}
                                max={100}
                                unit={"%"}
                                noTextBox={true}
                                noLabel={true}
                              />
                            </div>


                          <div hidden={double_slider_topic === null}>
                                <RangeAdjustment
                                  title="Double Image Slider"
                                  min={double_slider_values[0]}
                                  max={double_slider_values[1]}
                                  min_limit_m={0}
                                  max_limit_m={1}
                                  topic={namespace + "/set_range_window"}
                                  tooltip={"Adjustable range"}
                                  noTextBox={true}
                                />
                        
                            </div>
                    
                          {((single_slider_topic === true || double_slider_topic === true) && (show_save_controls === true ) && namespace !== 'None') ?
                            <div style={{ borderTop: "1px solid #ffffff", marginTop: Styles.vars.spacing.medium, marginBottom: Styles.vars.spacing.xs }}/>
                          : null }



      <div align={"left"} textAlign={"left"} hidden={( namespace === 'None')}>

       

                <div style={{ display: 'flex' }}>



                        <div style={{ width: '15%' }} >

                           { (show_info_controls === true ) ?
                            <Label title="Image Info">
                                {/* react-toggle (not AsyncToggle): checked is local view state, already immediate -- no backend round trip to confirm. */}
                                <Toggle
                                  checked={selected_control==='INFO'}
                                  onClick={() => onChangeChangeStateValue.bind(this)("selected_control",(selected_control === 'INFO') ? 'NONE' : 'INFO' )}>
                                </Toggle>
                              </Label>
                              : null }
                        </div>

                        <div style={{ width: '5%' }}>
                        </div>
                        

                        <div style={{ width: '15%' }}>

                              { (show_config_controls === true ) ?
                              <Label title="Image Config">
                                {/* react-toggle (not AsyncToggle): checked is local view state, already immediate -- no backend round trip to confirm. */}
                                <Toggle
                                  checked={selected_control==='CONFIG'}
                                  onClick={() => onChangeChangeStateValue.bind(this)("selected_control",(selected_control === 'CONFIG') ? 'NONE' : 'CONFIG' )}>
                                </Toggle>
                            </Label>
                            : null }
                        </div>

                        <div style={{ width: '5%' }}>
                        </div>

            

                       <div style={{ width: '15%' }} >
                              { (show_render_controls === true ) ?
                              <Label title="Render">
                                {/* react-toggle (not AsyncToggle): checked is local view state, already immediate -- no backend round trip to confirm. */}
                                <Toggle
                                  checked={selected_control==='RENDER'}
                                  onClick={() => onChangeChangeStateValue.bind(this)("selected_control",(selected_control === 'RENDER') ? 'NONE' : 'RENDER' )}>
                                </Toggle>
                            </Label>
                            : null }
                        </div>

                        <div style={{ width: '5%' }}>
                        </div>



                        <div style={{ width: '15%' }} >
                            { (show_navpose_controls === true ) ?
                            <Label title="NavPose">
                                {/* react-toggle (not AsyncToggle): checked is local view state, already immediate -- no backend round trip to confirm. */}
                                <Toggle
                                  checked={selected_control==='NAVPOSE'}
                                  onClick={() => onChangeChangeStateValue.bind(this)("selected_control",(selected_control === 'NAVPOSE') ? 'NONE' : 'NAVPOSE' )}>
                                </Toggle>
                              </Label>
                              : null }
                        </div>

                        <div style={{ width: '5%' }}>
                        </div>


                       <div style={{ width: '15%' }} >
                              { (show_overlay_controls === true ) ?
                              <Label title="Overlays">
                                {/* react-toggle (not AsyncToggle): checked is local view state, already immediate -- no backend round trip to confirm. */}
                                <Toggle
                                  checked={selected_control==='OVERLAY'}
                                  onClick={() => onChangeChangeStateValue.bind(this)("selected_control",(selected_control === 'OVERLAY') ? 'NONE' : 'OVERLAY' )}>
                                </Toggle>
                            </Label>
                            : null }
                        </div>

                      </div>




                  <div align={"left"} textAlign={"left"} hidden={(show_info !== true || namespace === 'None')}>


                        <div style={{ borderTop: "1px solid #ffffff", marginTop: Styles.vars.spacing.medium, marginBottom: Styles.vars.spacing.xs }}/>


                        <div style={{ display: 'flex' }}>
                          <div style={{ width: '40%' }}>


                          <Label title={"INFO"} />
                              {this.renderInfo()}


                          </div>

                          <div style={{ width: '20%' }}>
                            {}
                          </div>

                          <div style={{ width: '40%' }}>


                          <Label title={"STATS"} />
                              {this.renderStats()}


                          </div>
                        </div>




                  </div>

       


                  <div align={"left"} textAlign={"left"} hidden={show_overlay !== true || namespace === 'None'}>

                          <div style={{ borderTop: "1px solid #ffffff", marginTop: Styles.vars.spacing.medium, marginBottom: Styles.vars.spacing.xs }}/>

                          <div style={{ display: 'flex' }}>
                                <div style={{ width: '30%' }}>

                                      {(show_overlay_text === true) ? this.renderTextControls() : null}

                                </div>

                                <div style={{ width: '3%' }}>
                                  {}
                                </div>

                                <div style={{ width: '30%' }}>
                                   {(show_overlay_crosshairs === true) ? this.renderCrosshairsControls() : null}
                                </div>

                                <div style={{ width: '3%' }}>
                                  {}
                                </div>

                                <div style={{ width: '30%' }}>
                                   {(show_overlay_targets === true) ? this.renderTargetsControls() : null}

                                </div>
                
                          </div>

                            {(show_save_controls === true && config_topic !== '') ?
                            <NepiIFConfig
                                namespace={config_topic}
                                title={"Nepi_IF_Config"}
                            />
                            : null }


                    </div>


                  <div align={"left"} textAlign={"left"} hidden={(show_config !== true || namespace === 'None')}>

                          <div style={{ borderTop: "1px solid #ffffff", marginTop: Styles.vars.spacing.medium, marginBottom: Styles.vars.spacing.xs }}/>

                          <div style={{ display: 'flex' }}>
                                <div style={{ width: '30%' }}>

                          
                                  {this.renderFilterControls()}


                                </div>


                                <div style={{ width: '3%' }}>
                                  {}
                                </div>

                                <div style={{ width: '30%' }}>
                                  {this.renderAspectControls()}
                                </div>

                                <div style={{ width: '3%' }}>
                                  {}
                                </div>

                                <div style={{ width: '30%' }}>
  
                                  {this.renderORControls()}
                                </div>
                
                          </div>



                            {(show_save_controls === true && config_topic !== '') ?
                            <NepiIFConfig
                                namespace={config_topic}
                                title={"Nepi_IF_Config"}
                            />
                            : null }


                    </div>

                    <div align={"left"} textAlign={"left"} hidden={(show_navpose !== true || namespace === 'None')}>

                      {this.renderNavPoseSection()}


                            {(show_save_controls === true && namespace !== 'None') ?
                            <NepiIFConfig
                                namespace={namespace}
                                title={"Nepi_IF_Config"}
                            />
                            : null }

                    </div>



                  <div align={"left"} textAlign={"left"} hidden={(show_render !== true || namespace === 'None')}>

                          <div style={{ borderTop: "1px solid #ffffff", marginTop: Styles.vars.spacing.medium, marginBottom: Styles.vars.spacing.xs }}/>

                          <div style={{ display: 'flex' }}>
                                <div style={{ width: '40%' }}>

                                  {this.renderRenderControls()}


                                </div>

                                <div style={{ width: '20%' }}>
                                  {}
                                </div>

                                <div style={{ width: '40%' }}>
                                  {this.renderLiveControls()}
                                    {this.renderStreamControls()}
                
                                </div>
                              </div>

          

                            {(show_save_controls === true && config_topic !== '') ?
                            <NepiIFConfig
                                namespace={config_topic}
                                title={"Nepi_IF_Config"}
                            />
                            : null }

                      </div>

                          {(show_save_controls === true && namespace !== 'None') ?
                          <div style={{ borderTop: "1px solid #ffffff", marginTop: Styles.vars.spacing.medium, marginBottom: Styles.vars.spacing.xs }}/>
                          : null }
                            {(show_save_controls === true && namespace !== 'None') ?
                              <NepiIFSaveData
                              saveNamespace={save_data_topic}
                              make_section={false}
                              show_all_save_options={false}
                              show_topic_selector={show_topic_selector}
                              onSnapshotOverride={this.saveImageFrame}
                            />
                          : null }





{/* 
                            {(show_save_controls === true && config_topic !== '') ?
                            <NepiIFConfig
                                namespace={config_topic}
                                title={"Nepi_IF_Config"}
                            />
                            : null }
*/}



        </div>

        </Column>
        </Columns>
      

    )
  }


  render() {
    const make_section = (this.props.make_section !== undefined)? this.props.make_section : true
    if (make_section === false){
      return (
          <React.Fragment>
        
        {this.renderImageViewer()}
          </React.Fragment>
      )
    }
    else {
      return (

      <Section>
        {this.renderImageViewer()}

      </Section>
      )

    }
  }


}

Nepi_IF_ImageViewer.defaultProps = {
  imageRecognitions: [
    // {
    //   label: "foobar",
    //   roi: { x_offset: 500, y_offset: 100, width: 300, height: 400 }
    // }
  ]
}

export default Nepi_IF_ImageViewer