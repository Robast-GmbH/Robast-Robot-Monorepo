import { useState } from 'react'
import PropTypes from 'prop-types'




const AddOrder = ({ onAdd, coords }) => {
        const [orderItem, setOrderItem] = useState('')
        const [recurring_order, setRecurringOrder] = useState(false)

        const onSubmit = (e) => {
                e.preventDefault()
                if (!orderItem) {
                        alert('Add stupid orderItem')
                        return
                }

                onAdd({ orderItem, recurring_order, coords: {} })
                setOrderItem('')
                setRecurringOrder(false)
        }

        return (
                <form className='add-form' onSubmit={onSubmit}>
                        <div className='form-control'>
                                <label>Order</label>
                                <input type='orderItem' placeholder='What do you want?'
                                        value={orderItem} onChange={(e) => setOrderItem(e.target.value)} />
                        </div>

                        <div className='form-control form-control-checkbox'>
                                <label>Is it recurring?</label>
                                <input
                                        type='checkbox'
                                        checked={recurring_order}
                                        value={recurring_order}
                                        onChange={(e) => setRecurringOrder(e.currentTarget.checked)} />
                        </div>

                        <div className='form-control form-control-checkbox'>
                                <label>Goal-Coordinates: (debug)</label>
                                {coords ? (
                                        <h1>x:{coords.x}, y:{coords.y}</h1>
                                ) : (
                                        <h1>no coords</h1>
                                )}


                        </div>

                        <input type='submit' value='Save' className='btn btn-block' />
                </form>
        )
}

AddOrder.defaultProps = {
        coords: []
}
AddOrder.propTypes = {
        coords: PropTypes.object,
        onAdd: PropTypes.func
}
export default AddOrder

