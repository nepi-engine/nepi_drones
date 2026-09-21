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

import Styles from "./Styles"

const styles = Styles.Create({
  button: {
    border: "none",
    textAlign: "center",
    backgroundColor: Styles.vars.colors.grey0,
    cursor: "pointer",
    padding: `${Styles.vars.spacing.xs} ${Styles.vars.spacing.small}`,
    color: Styles.vars.colors.black
  },
  buttonDown: {
    border: "none",
    textAlign: "center",
    backgroundColor: Styles.vars.colors.blue,
    cursor: "pointer",
    padding: `${Styles.vars.spacing.xs} ${Styles.vars.spacing.small}`,
    color: Styles.vars.colors.black    
  },
  menuContainer: {
    marginTop: Styles.vars.spacing.small,
    textAlign: "right"
  }
})

class Button extends Component {
  constructor(props) {
    super(props)
  
    this.state = {
      actualStyle: {
        ...styles.button,
        ...this.props.style
      }
    }

    this.onButtonDown = this.onButtonDown.bind(this)
    this.onButtonUp = this.onButtonUp.bind(this)
  }

  onButtonDown() {
    const { style } = this.props
    this.setState({
      actualStyle: {
        ...styles.buttonDown,
        ...style
      }
    })
    if ( this.props.buttonDownAction !== null)
    {
      if (typeof this.props.buttonDownAction === 'function')
      {
        (this.props.buttonDownAction())
      }
  }
  }

  onButtonUp() {
    const { style } = this.props
    this.setState({
      actualStyle: {
        ...styles.button,
        ...style
      }
    })
    if (this.props.buttonUpAction !== null)
    {
      if (typeof this.props.buttonUpAction === 'function')
      {
        (this.props.buttonUpAction())
      }
    }
  }

  componentDidUpdate(prevProps, prevState, snapshot) {
    const prevBg = prevProps.style && prevProps.style.backgroundColor
    const nextBg = this.props.style && this.props.style.backgroundColor
    if (prevBg !== nextBg) {
      this.setState({
        actualStyle: this.props.style
      })
    }
  }

  render (){
    const { children } = this.props
    return (
      <button {...this.props} style={this.state.actualStyle} onMouseDown={this.onButtonDown} onMouseUp={this.onButtonUp}>
        {children}
      </button>
    )
  }
}

export default Button

const ButtonMenu = props => {
  const { style, children } = props
  const buttons = React.Children.toArray(children).filter(child => child)

  const actualStyle = {
    ...styles.menuContainer,
    ...style
  }

  return (
    <div {...props} style={actualStyle}>
      {buttons.map((button, index) =>
        // Merge, don't replace -- a button that sets its own style (e.g. a
        // selected-state backgroundColor) had that style silently discarded
        // by this clone, since it used to overwrite `style` outright.
        React.cloneElement(button, {
          style: { marginLeft: Styles.vars.spacing.regular, ...button.props.style }
        })
      )}
    </div>
  )
}

export { ButtonMenu }
