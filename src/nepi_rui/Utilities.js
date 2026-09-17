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


import React from "react"
import { Option } from "./Select"
import Styles from "./Styles"


/////////////////////////////
// MISC FUNCTIONS

export function doNothing(){
  var ret = false
  return ret
}

export function round(value, decimals = 0) {
  return Number(value).toFixed(decimals)
  //return value && Number(Math.round(value + "e" + decimals) + "e-" + decimals)
}

export function convertStrToStrList(inputStr) {
  var strList = []
  if (inputStr != null){
    inputStr = inputStr.replaceAll("[","")
    inputStr = inputStr.replaceAll("]","")
    inputStr = inputStr.replaceAll(" '","")
    inputStr = inputStr.replaceAll("'","")
    strList = inputStr.split(",")
  }
  return strList
}

export function filterStrList(inputList,filterList) {
  var outputList = []
  for (var i = 0; i < inputList.length; ++i) {
    var filter_check = false
    for (var j = 0; j < filterList.length; ++j) {
      if (inputList[i].indexOf(filterList[j]) !== -1) {
        filter_check = true
      }
    }
    if (filter_check === false){
      outputList.push(inputList[i])
    }
  }
  return outputList
}


export class Queue {
  constructor() {
      this.items = []
      this.frontIndex = 0
      this.backIndex = 0
  }
  pushItem(item) {
      this.items[this.backIndex] = item
      this.backIndex++
      return item + ' inserted'
  }
  pullItem() {
      var item = ""
      if (this.frontIndex < this.backIndex){
        item = this.items[this.frontIndex]
        delete this.items[this.frontIndex]
        this.frontIndex++
      }
      return item
  }
  clearItems(){
    var queue_length = this.getLength()
    while ( queue_length > 0 ){
      this.pullItem()
      queue_length = this.getLength()
    }
  }
  getItems(){
    return this.items.slice(this.frontIndex,this.backIndex)
  }
  getLength(){
    return this.backIndex - this.frontIndex
  }
  peek() {
      return this.items[this.frontIndex]
  }
  get printQueue() {
      return this.items;
  }
}

export function getCleanName(name, replacement = '_'){
  // Regex: 
  // <>:"/\\|?* : Reserved filename characters
  // \x00-\x1F : Control characters
  const invalidChars = /[<>:"/\\|?*\x00-\x1F]/g;
  return name.replace(invalidChars, replacement);
}


export function rgbToHex(r, g, b){
  // Ensure the inputs are clamped between 0 and 255
  const toHex = (c) => Math.min(255, Math.max(0, c)).toString(16).padStart(2, '0');
  return `#${toHex(r)}${toHex(g)}${toHex(b)}`;
}

export function rgbToIindicatorColor(r, g, b){
        const color_hex = rgbToHex(r, g, b)
      const indicator_color = {
        backgroundColor: color_hex,
        color: Styles.vars.colors.black
      }
  return indicator_color
}
/////////////////////////////
// TOGGLE FUNCTIONS

export function onChangeSwitchStateValue(stateVarNameStr,currentVal){
  var key = stateVarNameStr
  var value = currentVal === false
  var obj  = {}
  obj[key] = value
  this.setState(obj)
}

export function onChangeChangeStateValue(stateVarNameStr,newVal){
  var key = stateVarNameStr
  var value = newVal
  var obj  = {}
  obj[key] = value
  this.setState(obj)
}

export function onChangeSwitchStateNestedValue(parentKey, nestedKey, currentVal) {
  var value = currentVal === false;
  
  this.setState({
    [parentKey]: {
      ...this.state[parentKey],
      [nestedKey]: {
        ...this.state[parentKey][nestedKey],
        fixed: value
      }
    }
  });
}

/////////////////////////////
// MENU FUNCTIONS


export function createMenuBaseName(optionStr) {
    var menu_name = ''
    var parts = []
    var sliced_parts = []
    // Callers pass topic names straight out of status messages, which are null
    // until the first status arrives. Fall through to the default name instead
    // of throwing on the split.
    if (optionStr == null || typeof optionStr !== 'string') {
      return menu_name
    }
    parts = optionStr.split('/')
    if (parts.length > 3){
      sliced_parts = parts.slice(3);
      menu_name = sliced_parts[0]
    }
    else {
      menu_name = optionStr
    }

    return menu_name

}


export function createMenuBaseNames(optionsStrList) {
    var menu_names = []
    var menu_name = ''
    var i = 0
    if (optionsStrList == null) {
      return menu_names
    }
    for (i = 0; i < optionsStrList.length; ++i) {
      menu_name = createMenuBaseName(optionsStrList[i])
      menu_names.push(menu_name)
    }
    return menu_names
    
}

export function createMenuFirstLastName(optionStr) {
    var menu_name = 'None'
    var parts = []
    var sliced_parts = []
    // See createMenuBaseName. 'None' is already the declared default here, it
    // just never got reached because the split threw first.
    if (optionStr == null || typeof optionStr !== 'string') {
      return menu_name
    }
    parts = optionStr.split('/')
    if (parts.length > 3){
      sliced_parts = parts.slice(3); 
      menu_name = sliced_parts[0]
      if (sliced_parts.length > 1){
        menu_name = menu_name + '-' + sliced_parts.pop()
      }
    }
    else {
      menu_name = optionStr
    }
    
    return menu_name
    
}


export function createMenuFirstLastNames(optionsStrList) {
    var menu_names = []
    var menu_name = ''
    var i = 0
    if (optionsStrList == null) {
      return menu_names
    }
    for (i = 0; i < optionsStrList.length; ++i) {
      menu_name = createMenuFirstLastName(optionsStrList[i])
      menu_names.push(menu_name)
    }
    return menu_names
    
}


export function createMenuShortName(optionStr, filterOutList = [], removeLast = false) {
    var menu_name = 'None'
    var parts = []
    var sliced_parts = []
    parts = optionStr.split('/')
    if (parts.length > 3){
      sliced_parts = parts.slice(3); 
      var include_part = true
      var i = 0
      var i2 = 0
      for (i = 0; i < sliced_parts.length; ++i) {
        for (i2 = 0; i2 < filterOutList.length; ++i2) {
          if (sliced_parts[i]===filterOutList[i2]){
            include_part = false
          }
        }
        if ((i === (sliced_parts.length - 1)) && (removeLast === true)){
          include_part = false
        }
        if (include_part === true){
          if (i === 0 ){
            menu_name = sliced_parts[i]
          }
          else {
            menu_name = menu_name + '-' + sliced_parts[i]
          }
        }
      }
    }
    else {
      menu_name = optionStr
    }
      
    
    return menu_name
    
}


export function createMenuShortNames(optionsStrList, filterOutList = [], removeLast = false) {
    var menu_names = []
    var menu_name = ''
    var i = 0
    for (i = 0; i < optionsStrList.length; ++i) {
      menu_name = createMenuShortName(optionsStrList[i], filterOutList, removeLast)
      menu_names.push(menu_name)
    }
    return menu_names
    
}

export function removeStringFromMenuNames(optionsStrList, removeString) {
    var menu_names = []
    var menu_name = ''
    var i = 0
    for (i = 0; i < optionsStrList.length; ++i) {
      menu_name = optionsStrList[i].replace(removeString,'')
      menu_names.push(menu_name)
    }
    return menu_names
    
}




export function createShortValues(list) {
  var tokenizedList = []
  var depthsToShort = 2
  var shortList = []
  for (var i = 0; i < list.length; ++i) {
    tokenizedList.push(list[i].split("/").reverse())
  }
  // Now create the return list
  for (i = 0; i < tokenizedList.length; ++i) {
    shortList.push(tokenizedList[i].slice(0, depthsToShort).reverse().join("/"))
  }
  return shortList
}

export function createShortUniqueValues(list) {
  var tokenizedList = []
  var depthsToUnique = []
  var uniqueList = []
  for (var i = 0; i < list.length; ++i) {
    tokenizedList.push(list[i].split("/").reverse())
    depthsToUnique.push(1)
    const newItemIndex = tokenizedList.length - 1 
    for (var j = 0; j < tokenizedList.length - 1; ++j) {
      var currentTestDepth = 0
      while (tokenizedList[j][currentTestDepth] === tokenizedList[newItemIndex][currentTestDepth]) {
        currentTestDepth += 1
        if (currentTestDepth >= depthsToUnique[j]) {
          depthsToUnique[j] += 1
        }
        if (currentTestDepth >= depthsToUnique[newItemIndex]) {
          depthsToUnique[newItemIndex] += 1
        }
      }
    }
  }
  // Now create the return list
  for (i = 0; i < tokenizedList.length; ++i) {
    uniqueList.push(tokenizedList[i].slice(0, depthsToUnique[i]).reverse().join("/"))
  }
  return uniqueList
}


export function createShortValuesFromNamespaces(namespacesList) {
  const filterList = ['idx/', 'ptx/', 'rbx/' , 'lsx/', 'npx/', 'ai/' ]
  var tokenizedList = []
  var outputList = []
  var shortName = ''
  for (var i = 0; i < namespacesList.length; ++i) {
      tokenizedList = namespacesList[i].slice(1).split("/")
      var tokens_len = tokenizedList.length
      if(tokenizedList.length === 2){
        shortName = tokenizedList[1]
      }     
      else if(tokenizedList.length === 3){
        shortName = tokenizedList[2]
      }
      else if(tokenizedList.length === 4){
        shortName = tokenizedList[2] + "/" + tokenizedList[3]
      }
      else if(tokenizedList.length === 5){
        shortName = tokenizedList[2] + "/" + tokenizedList[3] + "/" + tokenizedList[4]
      }
      else if(tokenizedList.length > 5){
        shortName = tokenizedList[tokens_len-4]
        for (var i2 = (tokens_len-3); i2 < tokenizedList.length; ++i2) {
          shortName = shortName + "/" + tokenizedList[i2]
        }
      }
      else {
        shortName = namespacesList[i]
      }
      for (var i3 = 0; i2 < filterList.length; ++i3) {
        shortName = shortName.replace(filterList[i3],"")
      }
      outputList.push(shortName)
  }
  return outputList
}

export function createShortImagesFromNamespaces(baseNamespace,namespacesList) {
  const filterList = [baseNamespace + '/' , 'idx/', 'ptx/', 'rbx/' , 'lsx/', 'npx/', 'ai/' ]
  var outputList = []
  var shortName = ""
  for (var i = 0; i < namespacesList.length; ++i) {
      shortName = namespacesList[i]
      for (var i2 = 0; i2 < filterList.length; ++i2) {
        shortName = shortName.replace(filterList[i2],"")
      }
      outputList.push(shortName)
    }
  return outputList
}

export function createShortImageFromNamespace(baseNamespace,namespace) {
  const filterList = [baseNamespace + '/', 'idx/', 'ptx/', 'rbx/' , 'lsx/', 'npx/', 'ai/' ]
  var shortName = ""
  for (var i2 = 0; i2 < filterList.length; ++i2) {
    shortName = shortName.replace(filterList[i2],"")
  }
  return shortName
}


export function createMenuListFromStrList(optionsStrList, useShortNames, filterOut, prefixOptionsStrList, appendOptionsStrList, noOptionStr) {
  var menuList = []
  var filteredTopics = []
  var i
  if (optionsStrList.length === 0){
    if (noOptionStr !== undefined){
        menuList.push(<Option value={noOptionStr}>{noOptionStr}</Option>)
    }
  }
  else {
    if (filterOut) {
      for (i = 0; i < optionsStrList.length; i++) {
          if (filterOut.includes(optionsStrList[i]) === false){
            filteredTopics.push(String(optionsStrList[i]))
          }
      }
    }
    var unique_names = null
    if (useShortNames === true){
      unique_names = createShortValuesFromNamespaces(filteredTopics)
    } 
    else{
      unique_names = filteredTopics
    }
    
    for (i = 0; i < prefixOptionsStrList.length; i++) {
        let option = prefixOptionsStrList[i]
        menuList.push(<Option value={option}>{option}</Option>)
    }

    for (i = 0; i < filteredTopics.length; i++) {
      menuList.push(<Option value={filteredTopics[i]}>{unique_names[i]}</Option>)
    }

    for (i = 0; i < appendOptionsStrList.length; i++) {
      let option = appendOptionsStrList[i]
      menuList.push(<Option value={option}>{option}</Option>)
    }
  }

   return menuList
}



export function createMenuListFromStrLists(optionsStrList, namesStrList, prefixOptionsStrList, appendOptionsStrList, noOptionStr) {
  var menuList = []
 
  var i
  if (optionsStrList.length === 0){
    if (noOptionStr !== ''){
        menuList.push(<Option value={noOptionStr}>{noOptionStr}</Option>)
    }
  }
  else {   
    
    for (i = 0; i < prefixOptionsStrList.length; i++) {
        let option = prefixOptionsStrList[i]
        menuList.push(<Option value={option}>{option}</Option>)
    }

    for (i = 0; i < optionsStrList.length; i++) {
      menuList.push(<Option value={optionsStrList[i]}>{namesStrList[i]}</Option>)
    }

    for (i = 0; i < appendOptionsStrList.length; i++) {
      let option = appendOptionsStrList[i]
      menuList.push(<Option value={option}>{option}</Option>)
    }
  }

   return menuList
}


// Depriciated
//////////////////////////////////////


export function onDropdownSelectedSetState(event, stateVarStr) {
  var key = stateVarStr
  var value = event.target.value
  var obj  = {}
  obj[key] = value
  this.setState(obj)
}


export function onDropdownSelectedSendStr(event, namespace) {
  const {sendStringMsg} = this.props.ros
  const value = event.target.value
  sendStringMsg(namespace,value)
}

export function onDropdownSelectedSendIndex(event, namespace) {
  const {sendIntMsg} = this.props.ros
  const value = event.target.value
  if (value !== "None") {
    const index = event.target.selectedIndex
    sendIntMsg(namespace,index)
  }
}

export function onDropdownSelectedSendIndex8(event, namespace) {
  const {sendInt8Msg} = this.props.ros
  const value = event.target.value
  if (value !== "None") {
    const index = event.target.selectedIndex
    sendInt8Msg(namespace,index)
  }
}

export function onDropdownSelectedSendDriverOption(event, namespace) {
  const {sendUpdateStringMsg} = this.props.ros
  const driver_name = this.state.driver_name
  const option_str = event.target.value
  sendUpdateStringMsg(namespace, driver_name, option_str)
}

/////////////////////////////
// INPUT BOX FUNCTIONS

export function onUpdateSetStateValue(event,stateVarStr) {
  var key = stateVarStr
  var value = event.target.value
  var obj  = {}
  obj[key] = value
  this.setState(obj)
  document.getElementById(event.target.id).style.color = Styles.vars.colors.red
  this.render()
}

export function onUpdateSetStateNestedValue(event,stateVarStr) {
  var key = stateVarStr
  var value = event.target.value
  var obj  = {}
  obj[key] = value
  this.setState(obj)
  document.getElementById(event.target.id).style.color = Styles.vars.colors.red
  this.render()
}






export function onEnterSendIntValue(event, namespace) {
  const {sendIntMsg} = this.props.ros
  if(event.key === 'Enter'){
    const value = parseInt(event.target.value, 10)
    if (!isNaN(value)){
      sendIntMsg(namespace,value)
    }
    document.getElementById(event.target.id).style.color = Styles.vars.colors.black
  }
}


export function onEnterSendFloatValue(event, namespace) {
  const {sendFloatMsg} = this.props.ros
  if(event.key === 'Enter'){
    const value = parseFloat(event.target.value)
    if (!isNaN(value)){
      sendFloatMsg(namespace,value)
    }
    document.getElementById(event.target.id).style.color = Styles.vars.colors.black
  }
}

export function onEnterSetStateFloatValue(event, stateVarStr) {
  if(event.key === 'Enter'){
    const value = parseFloat(event.target.value)
    if (!isNaN(value)){
      var key = stateVarStr
      var obj  = {}
      obj[key] = value
      this.setState(obj)
    }
    document.getElementById(event.target.id).style.color = Styles.vars.colors.black
  }
}

export function onUpdateSetStateNestedFloatValue(component, event, stateVarStr) {
  const rawVal = event.target.value;
  const floatVal = parseFloat(rawVal);
  const value = isNaN(floatVal) ? rawVal : floatVal;

  const keys = stateVarStr.split('.');
  const lastKey = keys.pop();

  const newState = { ...component.state };
  let ref = newState;

  for (let key of keys) {
    ref[key] = { ...ref[key] };
    ref = ref[key];
  }

  ref[lastKey] = value;
  component.setState(newState);

  const input = document.getElementById(event.target.id);
  if (input) input.style.color = isNaN(floatVal) ? 'red' : 'black';
}

/////////////////////////////
// STYLE FUNCTIONS
export function setElementStyleModified(e) {
  e.style.color = Styles.vars.colors.red
  e.style.fontWeight = "bold"
}

export function clearElementStyleModified(e) {
  e.style.color = Styles.vars.colors.black
  e.style.fontWeight = "normal"
}


